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

from radar_reader import FrameReader              # noqa: E402
from ml_logger import MlCsvLogger                  # noqa: E402
from manifest_schema import (                      # noqa: E402
    Manifest, RadarMeta, ThermalMeta, SyncMeta, make_session_dir,
)
from send_config import send_config                # noqa: E402


# ── radar read loop ─────────────────────────────────────────────────────────

def _radar_loop(reader: FrameReader, logger: MlCsvLogger,
                stop_ns: int, stats: dict):
    """Minimal radar capture loop — no fall detection, no plotting."""
    frames = 0
    drops  = 0
    while time.monotonic_ns() < stop_ns:
        frame = reader.read_frame()
        if frame is None:
            drops += 1
            continue
        # No fall detection during pure recording — labels come post-hoc.
        logger.log(frame, fall_tids=set(), faint_tids=set(), fall_detector=None)
        frames += 1
    stats['frames'] = frames
    stats['drops']  = drops


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

    print(f'[REC] Recording for {args.duration:.1f}s ...')
    stats = {'frames': 0, 'drops': 0}
    try:
        _radar_loop(reader, logger, stop_ns, stats)
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
