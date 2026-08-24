"""``saferoom-read`` — live radar acquisition, detection and logging.

Opens the CLI and data UART ports, optionally sends a ``.cfg``, then runs the
frame loop: parse TLVs, feed the rule-based and (optionally) ML fall
detectors, log to CSV, and drive the visualizer.
"""

import argparse
import json
import os
import queue
import sys
import threading
import time

import numpy as np
import serial

from saferoom.radar.config import parse_boundary_box, send_config
from saferoom.radar.csv_log import CsvLogger
from saferoom.radar.detector import FallDetector
from saferoom.radar.tlv import DATA_BAUD, FrameReader, sensor_to_world
from saferoom.radar.viz import RadarWindow

try:
    from saferoom.radar.ml_log import MlCsvLogger, compute_pc_stats
    _ML_LOGGER_AVAILABLE = True
except ImportError:
    _ML_LOGGER_AVAILABLE = False

try:
    from saferoom.ml.inference import MlFallDetector
    _ML_INFERENCE_AVAILABLE = True
except ImportError:
    _ML_INFERENCE_AVAILABLE = False

try:
    from saferoom.ir.confirmer import IrConfirmer, IrConfirmerParams
    _IR_CONFIRMER_AVAILABLE = True
except ImportError:
    _IR_CONFIRMER_AVAILABLE = False


def _emit_fall_event(notifier, event_type: str, track: dict, t_mono_ns: int,
                     cr, reason: str, source: str = 'radar') -> None:
    """Send the fall/faint decision to the dashboard AND append to
    logs/fall_events.jsonl for offline thesis evaluation.

    `source` distinguishes the rule-based radar path ('radar') from the
    independent ML detector ('radar_ml') on the dashboard."""
    payload = {
        'tid': int(track.get('tid', -1)),
        'x': float(track.get('x', 0.0)),
        'y': float(track.get('y', 0.0)),
        'z': float(track.get('z', 0.0)),
        'vz': float(track.get('vz', 0.0)),
        'reason': reason,
    }
    if cr is not None:
        payload['ir_decision'] = cr.decision
        payload['ir_confidence'] = round(cr.confidence, 3)
        payload['ir_features'] = {k: (None if (isinstance(v, float) and (v != v))
                                       else round(v, 4))
                                   for k, v in cr.features.items()}
        payload['ir_rules'] = cr.rule_passed
        payload['ir_pre_available'] = cr.pre_available
        payload['ir_frames_used'] = cr.frames_used

    if notifier is not None:
        notifier.event(event_type, track_id=payload['tid'], source=source, **payload)

    # Append to JSONL for thesis comparison
    os.makedirs('logs', exist_ok=True)
    row = {
        't_mono_ns': int(t_mono_ns),
        't_wall': time.time(),
        'event_type': event_type,
        'payload': payload,
    }
    with open('logs/fall_events.jsonl', 'a', encoding='utf-8') as f:
        f.write(json.dumps(row) + '\n')


def start_ir_capture(args, notifier=None, want_panel=False, want_confirmer=False):
    """Start MLX90640 capture and wire it to the requested consumers.

    Returns (ir_cap, ir_queue, confirmer, ir_rot_k). Any of ir_cap/ir_queue/
    confirmer may be None. The frame callback forwards each rotated frame to:
      * the IR confirmer (fall fusion)      — only if want_confirmer
      * the dashboard thermal panel         — only if notifier is not None
      * the Qt live panel queue             — only if want_panel
    Thermal forwarding to the dashboard is throttled to ~8 Hz to keep the
    notifier queue light regardless of the MLX refresh rate.
    """
    ir_rot_k = (args.ir_rotate // 90) % 4
    try:
        from saferoom.ir.recorder import Mlx90640Capture
        ir_queue = queue.Queue(maxsize=2) if want_panel else None

        confirmer = None
        if want_confirmer:
            if (not args.no_confirmer) and _IR_CONFIRMER_AVAILABLE:
                confirmer = IrConfirmer(IrConfirmerParams())
                print('[IR] confirmer enabled — calibrating background...')
            elif args.no_confirmer:
                print('[IR] confirmer DISABLED (--no-confirmer)')
            else:
                print('[WARN] ir_confirmer not importable; confirmer off.')

        last_thermal = [0.0]  # mutable closure holder for throttle
        def _ir_push(frame_np, t_mono_ns):
            rotated = np.rot90(frame_np, k=ir_rot_k) if ir_rot_k else frame_np
            if confirmer is not None:
                confirmer.push(rotated, t_mono_ns)
            if notifier is not None:
                now = time.time()
                if now - last_thermal[0] >= 0.12:   # ~8 Hz to the dashboard
                    finite = np.isfinite(rotated)
                    valid_pct = (100.0 * float(finite.sum()) / finite.size
                                 if finite.size else 0.0)
                    notifier.thermal(rotated, valid_pct=valid_pct)
                    last_thermal[0] = now
            if ir_queue is not None:
                try:
                    while True:
                        ir_queue.get_nowait()
                except Exception:
                    pass
                try:
                    ir_queue.put_nowait((rotated, t_mono_ns))
                except Exception:
                    pass

        ir_cap = Mlx90640Capture(refresh_hz=args.ir_hz,
                                 frame_callback=_ir_push,
                                 store_frames=False)
        ir_cap.start()
        dest = []
        if want_panel:
            dest.append('panel')
        if notifier is not None:
            dest.append('dashboard')
        if confirmer is not None:
            dest.append('confirmer')
        print(f'[IR] capture active @ {args.ir_hz} Hz, rotation {args.ir_rotate}° '
              f'→ {", ".join(dest) or "no consumers"}')
        return ir_cap, ir_queue, confirmer, ir_rot_k
    except Exception as e:  # noqa: BLE001
        print(f'[WARN] Could not start IR capture ({e}). Continuing without IR.')
        return None, None, None, ir_rot_k


# ─────────────────────────────────────────────────────────────────────────────
# Frame reading loop (runs in main thread or background thread)
# ─────────────────────────────────────────────────────────────────────────────

def _read_loop(reader, fall_detector, logger, frame_queue, stop_event,
               sensor_h, sensor_t, use_plot, ml_detector=None, z_offset=0.0,
               confirmer=None, no_confirmer=False, notifier=None,
               bounds=None, sensor_xy=None):
    frame_count  = 0
    fall_count   = 0
    faint_count  = 0
    ml_fall_count = 0
    # Dashboard live-state helpers (only used when a notifier is attached).
    prev_presence = None          # None = unknown; forces first emit
    fps_times     = []            # rolling wall-clock stamps for fps estimate
    last_tracks_push = 0.0        # throttle track heartbeat to ~2 Hz
    print('\n--- Reading frames. Press Ctrl+C to stop. ---\n')
    try:
        while not stop_event.is_set():
            frame = reader.read_frame()
            if frame is None:
                print('[WARN] Failed to read frame — retrying...')
                time.sleep(0.1)
                continue

            # Transform point cloud from sensor frame to world frame (Z=0 = floor).
            for p in frame['points']:
                p['x'], p['y'], p['z'] = sensor_to_world(
                    p['x'], p['y'], p['z'], sensor_h, sensor_t)
                p['z'] += z_offset

            # Apply the same z_offset to tracks and heights (firmware world frame).
            # Tracks come pre-transformed by the firmware using sensorPosition.
            # If sensorPosition height is off, z_offset corrects both together.
            if z_offset != 0.0:
                for t in frame['tracks']:
                    t['z'] += z_offset
                for h in frame['heights'].values():
                    h['maxZ'] += z_offset
                    h['minZ'] += z_offset

            frame_count += 1
            tracks  = frame['tracks']
            heights = frame['heights']

            # Fall / faint detection
            fall_tids  = set()
            faint_tids = set()
            ml_fall_tids = set()
            active_tids = {t['tid'] for t in tracks}
            for t in tracks:
                tid = t['tid']
                h = heights.get(tid)  # None if no TLV 1012 data this frame
                if h is None:
                    # No height data — skip to avoid false falls from h=0 fallback.
                    fall_detector.bump_frame(tid)
                    continue
                # Ignore ghost tracks near standing persons: small vertical extent
                # AND above floor level. A fallen person is also small vertically
                # (0.07–0.15 m) but at floor level (maxZ < 0.8 m) — must pass through.
                _h_extent = h.get('maxZ', 0) - h.get('minZ', 0)
                _max_z    = h.get('maxZ', 0)
                if _h_extent < 0.25 and _max_z > 0.8:
                    fall_detector.bump_frame(tid)
                    continue
                # Ignore tracks with too few points when above floor level.
                # Ghost tracks have <10 points and maxZ > 0.8m.
                # A person on the floor (faint) also has few points but maxZ < 0.8m — must pass through.
                _n_pts = sum(1 for idx in frame['indices'] if idx == tid)
                if _n_pts < 10 and _max_z > 0.8:
                    fall_detector.bump_frame(tid)
                    continue
                is_fall, is_faint = fall_detector.update(tid, h, t)
                if is_fall:
                    fall_tids.add(tid)
                    fall_count += 1
                    print(f'\n*** FALL DETECTED — Track {tid} at '
                          f'({t["x"]:.2f}, {t["y"]:.2f}, {t["z"]:.2f}) m ***\n')
                    # ── IR fusion (spec 2026-05-19) ─────────────────────
                    if confirmer is not None and not no_confirmer:
                        if not confirmer.is_available():
                            decision_event = 'fall_failopen'
                            cr = None
                            reason = 'ir unavailable'
                        else:
                            cr = confirmer.evaluate(frame['t_mono_ns'])
                            if cr.decision == 'confirmed':
                                decision_event = 'fall_confirmed'
                            elif cr.decision == 'failopen':
                                decision_event = 'fall_failopen'
                            else:
                                decision_event = 'fall_candidate'
                            reason = cr.reason
                        _emit_fall_event(notifier, decision_event, t,
                                         frame['t_mono_ns'], cr, reason)
                    else:
                        # No confirmer in play — emit legacy single-tier fall_fast.
                        _emit_fall_event(notifier, 'fall_fast', t,
                                         frame['t_mono_ns'], None, 'rule tier-1')
                if is_faint:
                    faint_tids.add(tid)
                    faint_count += 1
                    print(f'\n*** FAINT DETECTED — Track {tid} at '
                          f'({t["x"]:.2f}, {t["y"]:.2f}, {t["z"]:.2f}) m ***\n')
                    _emit_fall_event(notifier, 'faint', t,
                                     frame['t_mono_ns'], None, 'faint tier-2')

                # ML detector (runs alongside rule-based, independent)
                if ml_detector is not None and h is not None:
                    pc = compute_pc_stats(frame['points'], frame['indices'], tid)
                    is_fall_ml, prob_ml = ml_detector.update(tid, t, h, pc)
                    if is_fall_ml:
                        ml_fall_tids.add(tid)
                        ml_fall_count += 1
                        print(f'\n[ML] FALL DETECTED — Track {tid} '
                              f'(p={prob_ml:.2f})\n')
                        _emit_fall_event(notifier, 'fall_confirmed', t,
                                         frame['t_mono_ns'], None,
                                         f'ml p={prob_ml:.2f}', source='radar_ml')

            fall_detector.cleanup_old_tracks(active_tids)
            if ml_detector is not None:
                ml_detector.cleanup_old_tracks(active_tids)

            # Console
            pres = '● PRESENT' if frame['presence'] else '○ empty  '
            pts_n = len(frame['points'])
            trk_strs = []
            for t in tracks:
                tid = t['tid']
                h = heights.get(tid) or {}
                max_z = h.get('maxZ', float('nan'))
                ref   = fall_detector.ref_maxz(tid)
                flag  = ' ⚠FALL' if tid in fall_tids else (' ⚠FAINT' if tid in faint_tids else '')
                if tid in ml_fall_tids:
                    flag += ' [ML⚠]'
                trk_strs.append(
                    f"T{tid}({t['x']:+.2f},{t['y']:+.2f}m "
                    f"maxZ={max_z:.2f} ref={ref:.2f} vz={t['vz']:+.2f}){flag}"
                )
            print(f"[{frame['frameNum']:06d}] {pres}  {pts_n:3d}pts  "
                  + (', '.join(trk_strs) if trk_strs else 'no tracks'))

            logger.log(frame, fall_tids, faint_tids, fall_detector)

            # ── Dashboard live state: presence + fps + track positions ──────
            if notifier is not None:
                now = time.time()
                fps_times.append(now)
                fps_times[:] = [x for x in fps_times if now - x < 2.0]
                fps = len(fps_times) / 2.0

                present = bool(frame['presence'])
                if present != prev_presence:
                    notifier.event('presence' if present else 'presence_lost',
                                   fps=round(fps, 1))
                    prev_presence = present

                # Throttle the position heartbeat to ~2 Hz to keep UART/CPU light.
                if now - last_tracks_push >= 0.5:
                    notifier.tracks(round(fps, 1), bounds, sensor_xy, tracks)
                    last_tracks_push = now

            if use_plot and not frame_queue.full():
                frame_queue.put_nowait((frame, fall_tids, faint_tids))

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        msg = f'\nStopped. Frames: {frame_count}, Falls: {fall_count}, Faints: {faint_count}'
        if ml_detector is not None:
            msg += f', ML falls: {ml_fall_count}'
        print(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Read and parse TLV output from IWR6843 3D People Tracking.'
    )
    parser.add_argument('--cli',  required=True, help='CLI serial port  (e.g. COM4 or /dev/ttyUSB0)')
    parser.add_argument('--data', required=True, help='Data serial port (e.g. COM3 or /dev/ttyUSB1)')
    parser.add_argument('--cfg',  default=None,  help='Path to .cfg file to send (optional)')
    parser.add_argument('--plot',   action='store_true', help='Show live 2D PyQtGraph visualization')
    parser.add_argument('--plot3d', action='store_true', help='Add 3D OpenGL panel (requires PyOpenGL)')
    parser.add_argument('--log',  default=None,
                        help='Path to CSV log file. If omitted, auto-saves to logs/session_YYYYMMDD_HHMMSS.csv')
    parser.add_argument('--frame-period', type=float, default=0.05,
                        help='Expected frame period in seconds (default: 0.05 = 50ms = 20fps)')
    parser.add_argument('--sensor-height', type=float, default=2.05,
                        help='Sensor height above floor in meters (default: 2.05)')
    parser.add_argument('--sensor-tilt', type=float, default=15.0,
                        help='Sensor downward tilt in degrees (default: 15.0)')
    parser.add_argument('--z-offset', type=float, default=0.0, metavar='METERS',
                        help='Z correction in metres applied to all points and tracks '
                             '(e.g. --z-offset -0.20 to shift down 20 cm). '
                             'Use to calibrate when objects appear at the wrong height.')
    # ML dataset collection
    parser.add_argument('--ml-log', action='store_true',
                        help='Use enhanced ML CSV logger (more columns: ax/ay/az, pc stats, label)')
    parser.add_argument('--label-mode', action='store_true',
                        help='Activate real-time keyboard labeling (requires --ml-log). '
                             'Keys: f=fall  n=normal  s=sitting  w=walking')
    # ML inference
    parser.add_argument('--ml-model', default=None, metavar='PATH',
                        help='Path to trained ML fall detector (.pkl or .pt). '
                             'Runs alongside rule-based detector when provided.')
    # Dashboard integration
    parser.add_argument('--dashboard', default=None, metavar='URL',
                        help='Dashboard base URL (e.g. http://localhost:8000) to POST '
                             'fall/faint events to. Rule-based falls -> fall_fast/'
                             'fall_confirmed/fall_failopen/fall_candidate; ML falls -> '
                             'fall_confirmed (source=radar_ml). Offline if omitted.')
    # IR live view (MLX90640)
    parser.add_argument('--ir', action='store_true',
                        help='Add live MLX90640 thermal panel to the visualizer '
                             '(requires --plot or --plot3d, and adafruit-circuitpython-mlx90640).')
    parser.add_argument('--no-confirmer', action='store_true',
                        help='Disable the IR fall confirmer even if --ir is on. '
                             'Used for radar-only baseline comparisons in evaluation.')
    parser.add_argument('--ir-hz', type=int, default=16,
                        choices=[1, 2, 4, 8, 16, 32],
                        help='MLX90640 refresh rate in Hz (default: 16)')
    parser.add_argument('--ir-rotate', type=int, default=90,
                        choices=[0, 90, 180, 270],
                        help='Rotate IR frame N degrees CCW before display '
                             '(default: 90, matches current mount). Use for '
                             'physically tilted cameras.')
    args = parser.parse_args()

    import datetime
    import os

    # Auto-generate log path
    if args.log is None:
        os.makedirs('logs', exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        prefix = 'ml_session' if args.ml_log else 'session'
        args.log = f'logs/{prefix}_{ts}.csv'
        print(f'Auto-logging to {args.log}')

    # Send config if provided
    if args.cfg:
        ok = send_config(args.cli, args.cfg)
        if not ok:
            print('[WARN] Config may have had errors — continuing anyway.')
        time.sleep(2.0)

    # Open data port
    try:
        data_ser = serial.Serial(args.data, baudrate=DATA_BAUD, timeout=2.0)
    except serial.SerialException as e:
        print(f'[ERROR] Cannot open data port {args.data}: {e}')
        sys.exit(1)
    print(f'Data port {args.data} opened at {DATA_BAUD} baud.')

    reader        = FrameReader(data_ser)
    fall_detector = FallDetector(frame_period_s=args.frame_period)

    # Choose logger
    if args.ml_log:
        if not _ML_LOGGER_AVAILABLE:
            print('[ERROR] ml_logger.py not found in tools/. Cannot use --ml-log.')
            sys.exit(1)
        session_id = os.path.splitext(os.path.basename(args.log))[0]
        logger = MlCsvLogger(args.log, session_id=session_id,
                              label_mode=args.label_mode)
    else:
        if args.label_mode:
            print('[WARN] --label-mode has no effect without --ml-log.')
        logger = CsvLogger(args.log)

    # ML fall detector (optional, runs alongside rule-based)
    ml_detector = None
    if args.ml_model:
        if not _ML_INFERENCE_AVAILABLE:
            print('[WARN] ml_inference.py not found. Ignoring --ml-model.')
        elif not _ML_LOGGER_AVAILABLE:
            print('[WARN] ml_logger.py not found (needed for compute_pc_stats). '
                  'Ignoring --ml-model.')
        else:
            try:
                ml_detector = MlFallDetector(args.ml_model)
            except Exception as e:
                print(f'[WARN] Could not load ML model: {e}')

    # Dashboard notifier (optional, non-blocking; drops events if server offline)
    notifier = None
    if args.dashboard:
        try:
            from saferoom.notify import Notifier
            notifier = Notifier(args.dashboard)
            print(f'[dashboard] posting fall/faint events to {args.dashboard}')
        except Exception as e:
            print(f'[WARN] Could not init dashboard notifier: {e}')

    # Room geometry for the dashboard 2D view. The sensor_to_world transform
    # places the sensor at the world origin, so sensor_xy = (0, 0).
    room_bounds = parse_boundary_box(args.cfg)
    sensor_xy = (0.0, 0.0)

    import queue
    frame_queue = queue.Queue(maxsize=4)
    stop_event  = threading.Event()

    if args.plot or args.plot3d:
        # Qt event loop must run in the main thread.
        # Frame reader runs in a daemon background thread.
        bbox = room_bounds

        # ── Optional IR live capture (panel + confirmer + dashboard) ──────
        ir_queue  = None
        ir_cap    = None
        ir_rot_k  = (args.ir_rotate // 90) % 4
        confirmer = None
        if args.ir:
            ir_cap, ir_queue, confirmer, ir_rot_k = start_ir_capture(
                args, notifier=notifier, want_panel=True, want_confirmer=True)

        window = RadarWindow(frame_queue, stop_event, boundary_box=bbox,
                             plot3d=args.plot3d,
                             ir_queue=ir_queue, ir_rotate=ir_rot_k)
        window.show()

        reader_thread = threading.Thread(
            target=_read_loop,
            args=(reader, fall_detector, logger, frame_queue, stop_event,
                  args.sensor_height, args.sensor_tilt, True),
            kwargs={
                'ml_detector': ml_detector,
                'z_offset': args.z_offset,
                'confirmer': confirmer if args.ir else None,
                'no_confirmer': args.no_confirmer,
                'notifier': notifier,
                'bounds': room_bounds,
                'sensor_xy': sensor_xy,
            },
            daemon=True,
        )
        reader_thread.start()

        try:
            window.exec()   # blocks until window is closed
        finally:
            stop_event.set()
            data_ser.close()
            logger.close()
            if ir_cap is not None:
                try:
                    ir_cap.stop_and_collect()
                except Exception:
                    pass
            if notifier is not None:
                notifier.shutdown()
    else:
        # Headless: no Qt panel and (by design) no IR fusion confirmer, but if
        # --ir is set we still capture thermal frames and forward them to the
        # dashboard so the Live thermal panel works during a live --dashboard run.
        ir_cap = None
        if args.ir:
            ir_cap, _iq, _cf, _rk = start_ir_capture(
                args, notifier=notifier, want_panel=False, want_confirmer=False)
        try:
            _read_loop(reader, fall_detector, logger, frame_queue, stop_event,
                       args.sensor_height, args.sensor_tilt, False,
                       ml_detector=ml_detector, z_offset=args.z_offset,
                       confirmer=None, no_confirmer=args.no_confirmer,
                       notifier=notifier, bounds=room_bounds,
                       sensor_xy=sensor_xy)
        finally:
            data_ser.close()
            logger.close()
            if ir_cap is not None:
                try:
                    ir_cap.stop_and_collect()
                except Exception:
                    pass
            if notifier is not None:
                notifier.shutdown()


if __name__ == '__main__':
    main()
