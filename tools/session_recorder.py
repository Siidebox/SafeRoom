#!/usr/bin/env python3
"""
session_recorder.py — Synchronized radar + IR session recorder.

Runs in a single process on the Raspberry Pi 5. Uses time.monotonic_ns() as
the shared clock for both modalities so post-hoc alignment is trivial.

Output layout:
    sessions/<YYYYMMDD_HHMMSS>[_name]/
        radar.csv      — MlCsvLogger (one row per track per frame)
        thermal.npz    — MLX90640 frames + per-frame t_mono_ns + t_wall
        manifest.json  — metadata (durations, real fps, labels, ...)

Usage:
    python tools/session_recorder.py \
        --cli COM4 --data COM3 \
        --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
        --duration 60 --name banco_test_01 --ir-hz 16

Pass --no-ir to record radar only (useful when the MLX90640 isn't wired yet).
"""

import argparse
import os
import sys
import time

import numpy as np
import serial

# tools/ is on sys.path when invoked as `python tools/session_recorder.py`
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from radar_reader import FrameReader, FallDetector  # noqa: E402
from ml_logger import MlCsvLogger                  # noqa: E402
from manifest_schema import (                      # noqa: E402
    Manifest, RadarMeta, ThermalMeta, SyncMeta, make_session_dir,
)
from send_config import send_config                # noqa: E402


# ── radar read loop ─────────────────────────────────────────────────────────

def _detect_frame(frame: dict, fall_detector: FallDetector,
                  t0_mono: int) -> tuple:
    """Run the two-tier fall detector on one frame (same veto logic as
    radar_reader._read_loop). Prints a console alert on Tier-1 fall / faint.
    Returns (fall_tids, faint_tids). Detection is informative only — the CSV
    labels used for training come post-hoc, not from here."""
    tracks  = frame['tracks']
    heights = frame['heights']
    fall_tids  = set()
    faint_tids = set()
    active_tids = {t['tid'] for t in tracks}
    for t in tracks:
        tid = t['tid']
        h = heights.get(tid)              # None if no TLV 1012 this frame
        if h is None:
            fall_detector.bump_frame(tid)
            continue
        _h_extent = h.get('maxZ', 0) - h.get('minZ', 0)
        _max_z    = h.get('maxZ', 0)
        # Ghost track near a standing person: small vertical extent above floor.
        if _h_extent < 0.25 and _max_z > 0.8:
            fall_detector.bump_frame(tid)
            continue
        # Too few points while above floor level → ghost.
        _n_pts = sum(1 for idx in frame['indices'] if idx == tid)
        if _n_pts < 10 and _max_z > 0.8:
            fall_detector.bump_frame(tid)
            continue
        is_fall, is_faint = fall_detector.update(tid, h, t)
        t_rel = (time.monotonic_ns() - t0_mono) / 1e9
        if is_fall:
            fall_tids.add(tid)
            peak_vz = fall_detector._last_peak_vz.get(tid, 0.0)
            print(f'[REC]  t={t_rel:5.1f}s  >> CAIDA Tier-1  tid={tid}  '
                  f'vz={peak_vz:+.2f} m/s  pos=({t["x"]:.2f},{t["y"]:.2f})',
                  flush=True)
        if is_faint:
            faint_tids.add(tid)
            print(f'[REC]  t={t_rel:5.1f}s  >> INMOVIL/FAINT  tid={tid}',
                  flush=True)
    fall_detector.cleanup_old_tracks(active_tids)
    return fall_tids, faint_tids


def _radar_loop(reader: FrameReader, logger: MlCsvLogger,
                stop_ns: int, stats: dict,
                fall_detector: FallDetector = None, t0_mono: int = 0):
    """Radar capture loop. With fall_detector set (--live-fall), runs the
    two-tier detector inline and prints console alerts; otherwise pure record."""
    frames = 0
    drops  = 0
    falls  = 0
    while time.monotonic_ns() < stop_ns:
        frame = reader.read_frame()
        if frame is None:
            drops += 1
            continue
        if fall_detector is not None:
            fall_tids, faint_tids = _detect_frame(frame, fall_detector, t0_mono)
            falls += len(fall_tids)
        else:
            fall_tids, faint_tids = set(), set()
        logger.log(frame, fall_tids=fall_tids, faint_tids=faint_tids,
                   fall_detector=fall_detector)
        frames += 1
    stats['frames'] = frames
    stats['drops']  = drops
    stats['falls']  = falls


# ── main ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--cli',  required=True, help='CLI serial port (e.g. /dev/ttyUSB0)')
    ap.add_argument('--data', required=True, help='Data serial port (e.g. /dev/ttyUSB1)')
    ap.add_argument('--cfg',  required=True, help='Radar .cfg file to send before capture')
    ap.add_argument('--duration', type=float, default=60.0,
                    help='Recording duration in seconds (default: 60)')
    ap.add_argument('--name', default='', help='Optional session name suffix')
    ap.add_argument('--subject', default='', help='Subject identifier (for manifest)')
    ap.add_argument('--notes', default='', help='Free-text notes (for manifest)')
    ap.add_argument('--ir-hz', type=int, default=16,
                    help='MLX90640 refresh rate; 0 disables IR (default: 16)')
    ap.add_argument('--no-ir', action='store_true',
                    help='Skip IR capture entirely (radar-only session)')
    ap.add_argument('--sessions-dir', default='sessions',
                    help='Base directory for session folders (default: sessions/)')
    ap.add_argument('--live-fall', action='store_true',
                    help='Run the two-tier fall detector inline and print a '
                         'console alert on Tier-1 falls (informative; labels '
                         'still come post-hoc). No GUI, minimal fps impact.')
    args = ap.parse_args()

    use_ir = (not args.no_ir) and (args.ir_hz > 0)

    # ── set up output directory ──
    session_dir = make_session_dir(args.sessions_dir, args.name)
    session_id  = os.path.basename(session_dir)
    radar_csv   = os.path.join(session_dir, 'radar.csv')
    thermal_npz = os.path.join(session_dir, 'thermal.npz')
    print(f'[REC] Session: {session_dir}')

    # ── radar: send config + open data port ──
    print(f'[REC] Sending config {args.cfg} via {args.cli} ...')
    ok = send_config(args.cli, args.cfg, verbose=False)
    if not ok:
        print('[REC] CLI config failed. Aborting.')
        sys.exit(1)

    data_port = serial.Serial(args.data, 921600, timeout=2.0)
    # Drop any TLV bytes the kernel queued before our config completed; the
    # radar firmware may have been streaming from a previous session and
    # would otherwise force the magic-word sync to scan stale data.
    time.sleep(0.15)
    data_port.reset_input_buffer()
    reader = FrameReader(data_port)
    logger = MlCsvLogger(radar_csv, session_id=session_id, label_mode=False)

    # ── IR: start capture in background ──
    cap = None
    if use_ir:
        try:
            from ir_recorder import Mlx90640Capture
            cap = Mlx90640Capture(refresh_hz=args.ir_hz)
            cap.start()
            print(f'[REC] IR capture started @ {args.ir_hz} Hz')
        except Exception as e:  # noqa: BLE001
            print(f'[REC] IR init failed ({e}). Continuing radar-only.')
            cap = None

    # ── run ──
    t0_mono = time.monotonic_ns()
    t0_wall = time.strftime('%Y-%m-%dT%H:%M:%S%z') or time.strftime('%Y-%m-%dT%H:%M:%S')
    stop_ns = t0_mono + int(args.duration * 1e9)

    fall_detector = FallDetector(frame_period_s=0.05) if args.live_fall else None
    if fall_detector is not None:
        print('[REC] Live fall detection ON (Tier-1 vz <= -1.15 m/s)')

    print(f'[REC] Recording for {args.duration:.1f}s ...')
    stats = {'frames': 0, 'drops': 0, 'falls': 0}
    try:
        _radar_loop(reader, logger, stop_ns, stats,
                    fall_detector=fall_detector, t0_mono=t0_mono)
    except KeyboardInterrupt:
        print('\n[REC] Interrupted')

    duration_s = (time.monotonic_ns() - t0_mono) / 1e9
    logger.close()
    data_port.close()

    # ── collect IR ──
    ir_frames_n = 0
    ir_bad      = 0
    ir_fps      = 0.0
    if cap is not None:
        frames, t_mono, t_wall, bad = cap.stop_and_collect()
        ir_frames_n = int(len(frames))
        ir_bad      = int(bad)
        ir_fps      = ir_frames_n / duration_s if duration_s > 0 else 0.0
        np.savez(
            thermal_npz,
            frames=frames,
            timestamps=t_wall,
            t_mono_ns=t_mono,
            refresh_rate_hz=args.ir_hz,
            bad_frames=ir_bad,
        )
        print(f'[REC] IR: {ir_frames_n} frames, {ir_bad} bad, {ir_fps:.2f} fps')

    radar_fps = stats['frames'] / duration_s if duration_s > 0 else 0.0
    print(f'[REC] Radar: {stats["frames"]} frames, {stats["drops"]} drops, '
          f'{radar_fps:.2f} fps')
    if fall_detector is not None:
        print(f'[REC] Live fall detection: {stats["falls"]} Tier-1 alert(s)')

    # ── manifest ──
    # rows == frames here because the logger writes one row per frame even when
    # there are no tracks (presence-only row). With multiple tracks per frame
    # we'd count rows via csv reread; not needed for now.
    m = Manifest(
        session_id=session_id,
        started_at_wall=t0_wall,
        started_at_mono_ns=t0_mono,
        duration_s=duration_s,
        subject=args.subject,
        notes=args.notes,
        radar=RadarMeta(
            cfg_file=os.path.basename(args.cfg),
            fps_target=20.0,
            fps_real=radar_fps,
            rows=stats['frames'],
            frames=stats['frames'],
            drops=stats['drops'],
        ),
        thermal=ThermalMeta(
            refresh_hz=args.ir_hz if use_ir else 0,
            fps_real=ir_fps,
            frames=ir_frames_n,
            bad_frames=ir_bad,
            shape=[24, 32],
        ),
        sync=SyncMeta(
            clock='monotonic_ns',
            note='radar t_mono_ns captured at magic-word sync; '
                 'IR t_mono_ns captured at end of each getFrame()',
        ),
    )
    m.write(session_dir)
    print(f'[REC] Manifest written: {os.path.join(session_dir, "manifest.json")}')


if __name__ == '__main__':
    main()
