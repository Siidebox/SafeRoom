#!/usr/bin/env python3
"""
synth_session.py — Generate a synthetic SafeRoom session for pipeline testing.

Produces a structurally valid sessions/<id>/ folder (radar.csv + thermal.npz +
manifest.json) without any hardware. Used to validate feature_engineering →
train_model → evaluate_model end-to-end before real data arrives.

The "story" of the synthetic session:
    - Subject walks for the first half of the recording (standing, vz ≈ 0,
      maxZ ≈ 1.7 m, modest horizontal velocity).
    - Around the middle, a fall: vz drops to −1.5 m/s for ~4 frames,
      maxZ collapses from 1.7 m to 0.3 m, then person stays on the floor.

The IR stream mirrors this: a 2D gaussian "hot blob" that moves around and
then drops vertically in the frame at the fall instant.
"""

import argparse
import csv
import os
import time

import numpy as np

from saferoom.dataset.manifest import (
    LabelSpan,
    Manifest,
    RadarMeta,
    SyncMeta,
    ThermalMeta,
    make_session_dir,
)
from saferoom.radar.ml_log import ML_COLUMNS

RADAR_FPS = 20.0
IR_FPS    = 8.0   # effective fps with refresh_hz=16


# ── radar.csv synthesis ─────────────────────────────────────────────────────

def _gen_radar(n_frames: int, t0_mono_ns: int, fall_frame: int,
               session_id: str, rng: np.random.Generator):
    """
    Yield (t_mono_ns, row) tuples for one synthetic track.

    Trajectory:
        0..fall_frame-15      : walking, maxZ≈1.7, vz≈0
        fall_frame-15..fall_frame+5 : falling, maxZ→0.3, vz dip to −1.5
        after                  : on floor, maxZ≈0.3, vz≈0
    """
    period_ns = int(1e9 / RADAR_FPS)

    for k in range(n_frames):
        t_mono = t0_mono_ns + k * period_ns
        t_wall = time.time() + k / RADAR_FPS

        # Fall is a smooth cosine ramp from frame fall_frame-15 to fall_frame+5
        if k < fall_frame - 15:
            phase = 'walk'
        elif k < fall_frame + 5:
            phase = 'fall'
        else:
            phase = 'floor'

        if phase == 'walk':
            maxZ = 1.70 + rng.normal(0, 0.03)
            vz   = rng.normal(0, 0.05)
            vx   = 0.4 + rng.normal(0, 0.05)
            vy   = rng.normal(0, 0.1)
            az   = rng.normal(0, 0.3)
        elif phase == 'fall':
            t = (k - (fall_frame - 15)) / 20.0       # 0 .. 1
            maxZ = 1.70 - (1.70 - 0.30) * t + rng.normal(0, 0.05)
            vz   = -1.5 * np.sin(np.pi * t) + rng.normal(0, 0.1)
            vx   = 0.1 + rng.normal(0, 0.1)
            vy   = rng.normal(0, 0.1)
            az   = -3.0 * np.cos(np.pi * t) + rng.normal(0, 0.3)
        else:  # floor
            maxZ = 0.30 + rng.normal(0, 0.02)
            vz   = rng.normal(0, 0.04)
            vx   = rng.normal(0, 0.05)
            vy   = rng.normal(0, 0.05)
            az   = rng.normal(0, 0.2)

        minZ = max(0.0, maxZ - 0.40 + rng.normal(0, 0.03))
        height_m = maxZ - minZ
        x = 0.0 + 0.4 * np.sin(k / 30.0)
        y = 2.0 + 0.3 * np.cos(k / 40.0)
        z = (maxZ + minZ) / 2.0
        ax = rng.normal(0, 0.4)
        ay = rng.normal(0, 0.4)
        confidence = 0.85 + rng.normal(0, 0.05)
        g = 0.9 + rng.normal(0, 0.03)

        n_pts = max(5, int(rng.normal(35 if phase != 'floor' else 18, 5)))
        pc_mean_doppler = -vz + rng.normal(0, 0.1)
        pc_std_doppler  = 0.4 + rng.normal(0, 0.05)
        pc_mean_snr     = 200 + rng.normal(0, 20)
        pc_std_snr      = 50 + rng.normal(0, 10)
        pc_mean_z       = z + rng.normal(0, 0.05)
        pc_std_z        = 0.15 + rng.normal(0, 0.02)
        pc_z_span       = height_m + rng.normal(0, 0.05)

        label = 'fall' if phase == 'fall' else (
            'lie' if phase == 'floor' else 'walk'
        )

        # Match ML_COLUMNS ordering exactly:
        # t_mono_ns, timestamp, session_id, frameNum, presence,
        # tid, x, y, z, vx, vy, vz, ax, ay, az, g, confidence,
        # height_m, maxZ, minZ, maxZ_ref, peak_vz,
        # fall_detected, faint_detected,
        # pc_n_points, pc_mean_doppler, pc_std_doppler,
        # pc_mean_snr, pc_std_snr, pc_mean_z, pc_std_z, pc_z_span,
        # num_total_points, num_tracks, label
        row = [
            t_mono, t_wall, session_id, k, 1,
            1,
            round(x, 4), round(y, 4), round(z, 4),
            round(vx, 4), round(vy, 4), round(vz, 4),
            round(ax, 4), round(ay, 4), round(az, 4),
            round(g, 4), round(confidence, 4),
            round(height_m, 4), round(maxZ, 4), round(minZ, 4), '', '',
            1 if phase == 'fall' else 0, 0,
            n_pts,
            round(pc_mean_doppler, 4), round(pc_std_doppler, 4),
            round(pc_mean_snr, 4), round(pc_std_snr, 4),
            round(pc_mean_z, 4), round(pc_std_z, 4), round(pc_z_span, 4),
            n_pts, 1,
            label,
        ]
        yield t_mono, row


# ── thermal.npz synthesis ───────────────────────────────────────────────────

def _gen_thermal(duration_s: float, t0_mono_ns: int, fall_t_mono_ns: int,
                 rng: np.random.Generator):
    """Hot 2D gaussian that walks and then drops in the frame at fall time."""
    period_ns = int(1e9 / IR_FPS)
    n = int(duration_s * IR_FPS)
    H, W = 24, 32
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    frames = np.zeros((n, H, W), dtype=np.float32)
    t_mono = np.zeros(n, dtype=np.int64)
    t_wall = np.zeros(n, dtype=np.float64)

    base_temp = 22.0  # ambient °C
    for k in range(n):
        ts = t0_mono_ns + k * period_ns
        t_mono[k] = ts
        t_wall[k] = time.time() + k / IR_FPS

        if ts < fall_t_mono_ns - int(0.75 * 1e9):
            cy = 6.0 + 1.5 * np.sin(k / 8.0)
            sigma = 3.5
        elif ts < fall_t_mono_ns + int(0.25 * 1e9):
            frac = (ts - (fall_t_mono_ns - int(0.75 * 1e9))) / 1e9
            cy = 6.0 + (18.0 - 6.0) * np.clip(frac, 0, 1)
            sigma = 3.5 + 1.0 * np.clip(frac, 0, 1)
        else:
            cy = 18.0 + rng.normal(0, 0.3)
            sigma = 4.5
        cx = 16.0 + 4.0 * np.sin(k / 12.0)

        blob = 13.0 * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * sigma ** 2))
        noise = rng.normal(0, 0.15, size=(H, W)).astype(np.float32)
        frames[k] = base_temp + blob + noise

    return frames, t_mono, t_wall


# ── main ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--out', default='', help='Session output directory '
                    '(default: sessions/<timestamp>_synth)')
    ap.add_argument('--duration', type=float, default=30.0,
                    help='Synthetic duration in seconds (default: 30)')
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        session_dir = args.out
    else:
        session_dir = make_session_dir('sessions', 'synth')
    session_id = os.path.basename(session_dir)

    n_frames   = int(args.duration * RADAR_FPS)
    fall_frame = n_frames // 2
    t0_mono    = time.monotonic_ns()
    period_ns  = int(1e9 / RADAR_FPS)
    fall_t_mono = t0_mono + fall_frame * period_ns

    # ── radar.csv ──
    radar_csv = os.path.join(session_dir, 'radar.csv')
    with open(radar_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(ML_COLUMNS)
        for _, row in _gen_radar(n_frames, t0_mono, fall_frame, session_id, rng):
            w.writerow(row)

    # ── thermal.npz ──
    thermal_npz = os.path.join(session_dir, 'thermal.npz')
    frames, t_mono, t_wall = _gen_thermal(args.duration, t0_mono, fall_t_mono, rng)
    np.savez(
        thermal_npz,
        frames=frames,
        timestamps=t_wall,
        t_mono_ns=t_mono,
        refresh_rate_hz=16,
        bad_frames=0,
    )

    # ── manifest.json ──
    fall_window_start = t0_mono + (fall_frame - 15) * period_ns
    fall_window_end   = t0_mono + (fall_frame + 5)  * period_ns
    m = Manifest(
        session_id=session_id,
        started_at_wall=time.strftime('%Y-%m-%dT%H:%M:%S'),
        started_at_mono_ns=t0_mono,
        duration_s=args.duration,
        subject='synthetic',
        notes='Generated by synth_session.py for pipeline validation only.',
        radar=RadarMeta(cfg_file='(synthetic)', fps_target=20.0,
                        fps_real=20.0, rows=n_frames,
                        frames=n_frames, drops=0),
        thermal=ThermalMeta(refresh_hz=16, fps_real=IR_FPS,
                            frames=len(frames), bad_frames=0,
                            shape=[24, 32]),
        sync=SyncMeta(clock='monotonic_ns', max_drift_ms_observed=0.0,
                      note='Synthetic — radar/IR share the same clock by construction'),
        labels=[
            LabelSpan(
                t_start_mono_ns=int(fall_window_start),
                t_end_mono_ns=int(fall_window_end),
                label='fall',
                notes='Synthetic fall event',
            )
        ],
    )
    m.write(session_dir)
    print(f'[SYNTH] session_dir={session_dir}')
    print(f'[SYNTH] radar.csv frames={n_frames}  thermal.npz frames={len(frames)}  '
          f'fall_frame={fall_frame}')


if __name__ == '__main__':
    main()
