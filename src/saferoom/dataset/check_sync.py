#!/usr/bin/env python3
"""
check_sync.py — Validate radar-IR temporal sync of a recorded session.

Procedure (for the bench smoke test described in the plan):
  1. Record a 60s session with session_recorder.py while waving a hand in
     front of both sensors approximately every 10s.
  2. Run this script on the session directory.
  3. It detects motion peaks in each modality and reports the cross-stream
     time offset (drift) at each event.

Radar motion proxy : |vz| (or, if absent, |vx| + |vy| from row-to-row
                     position diff). Peaks above a robust threshold are
                     events.
IR motion proxy    : frame-to-frame mean absolute temperature delta. Peaks
                     above the robust threshold are events.

Output:
  - Per-event matched pairs (radar_t, ir_t, delta_ms)
  - Mean / max / std drift
  - Pass/fail vs --max-drift-ms threshold (default 50 ms)

The script also writes the observed drift back into manifest.json under
sync.max_drift_ms_observed.

Usage:
    python tools/check_sync.py sessions/<session_id>/
    python tools/check_sync.py sessions/<session_id>/ --max-drift-ms 50 --plot
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd

from saferoom.dataset.manifest import Manifest


def _robust_peaks(signal: np.ndarray, t: np.ndarray,
                  min_gap_s: float = 3.0) -> np.ndarray:
    """
    Detect local maxima of `signal` above (median + 4·MAD), enforcing a
    minimum gap of `min_gap_s` between consecutive peaks.

    Returns the t-values (same units as `t`) of the peaks.
    """
    if len(signal) < 3:
        return np.empty(0)
    med = np.median(signal)
    mad = np.median(np.abs(signal - med)) + 1e-9
    thresh = med + 4.0 * mad
    above = signal > thresh
    if not above.any():
        # Fallback: top 1% only
        cutoff = np.percentile(signal, 99)
        above = signal >= cutoff

    # Local maxima
    peaks = []
    last_t = -np.inf
    for i in range(1, len(signal) - 1):
        if (above[i]
                and signal[i] >= signal[i - 1]
                and signal[i] >= signal[i + 1]
                and (t[i] - last_t) >= min_gap_s):
            peaks.append(t[i])
            last_t = t[i]
    return np.array(peaks)


def _radar_motion(df: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Return (t_seconds_since_start, motion_signal)."""
    if 't_mono_ns' not in df.columns:
        raise ValueError('radar.csv has no t_mono_ns column — cannot sync.')
    t = df['t_mono_ns'].to_numpy(dtype=np.int64)
    t0 = int(t.min())
    ts = (t - t0) / 1e9

    if 'vz' in df.columns:
        sig = df['vz'].fillna(0).abs().to_numpy(dtype=np.float32)
    elif {'x', 'y'}.issubset(df.columns):
        x = df['x'].fillna(0).to_numpy(dtype=np.float32)
        y = df['y'].fillna(0).to_numpy(dtype=np.float32)
        sig = np.zeros_like(x)
        sig[1:] = np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2)
    else:
        raise ValueError('radar.csv lacks motion columns (vz / x,y).')
    return ts, sig, t0


def _ir_motion(npz_path: str, t0_mono: int) -> tuple[np.ndarray, np.ndarray]:
    npz = np.load(npz_path)
    frames = npz['frames']
    if 't_mono_ns' not in npz.files:
        raise ValueError('thermal.npz has no t_mono_ns array — cannot sync.')
    t_mono = npz['t_mono_ns'].astype(np.int64)
    ts = (t_mono - t0_mono) / 1e9
    if len(frames) < 2:
        return ts, np.zeros(len(ts), dtype=np.float32)
    diffs = np.mean(np.abs(np.diff(frames.astype(np.float32), axis=0)),
                    axis=(1, 2))
    sig = np.concatenate([[0.0], diffs]).astype(np.float32)
    return ts, sig


def _match_peaks(radar_peaks: np.ndarray, ir_peaks: np.ndarray,
                 window_s: float = 1.0) -> list[tuple[float, float]]:
    """For each radar peak, find the nearest IR peak within ±window_s."""
    matched = []
    for rp in radar_peaks:
        if len(ir_peaks) == 0:
            break
        idx = int(np.argmin(np.abs(ir_peaks - rp)))
        if abs(ir_peaks[idx] - rp) <= window_s:
            matched.append((rp, float(ir_peaks[idx])))
    return matched


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('session_dir')
    ap.add_argument('--max-drift-ms', type=float, default=50.0)
    ap.add_argument('--match-window-s', type=float, default=1.0,
                    help='Max time gap to consider two peaks as the same event '
                         '(default: 1.0 s)')
    ap.add_argument('--plot', action='store_true',
                    help='Show diagnostic plot (radar motion, IR motion, peaks)')
    args = ap.parse_args()

    radar_csv   = os.path.join(args.session_dir, 'radar.csv')
    thermal_npz = os.path.join(args.session_dir, 'thermal.npz')
    if not os.path.isfile(radar_csv):
        sys.exit(f'Missing radar.csv in {args.session_dir}')
    if not os.path.isfile(thermal_npz):
        sys.exit(f'Missing thermal.npz in {args.session_dir}')

    df = pd.read_csv(radar_csv)
    radar_t, radar_sig, t0_mono = _radar_motion(df)
    ir_t, ir_sig = _ir_motion(thermal_npz, t0_mono)

    radar_peaks = _robust_peaks(radar_sig, radar_t)
    ir_peaks    = _robust_peaks(ir_sig,    ir_t)
    print(f'[SYNC] Radar peaks: {len(radar_peaks)}  IR peaks: {len(ir_peaks)}')

    matched = _match_peaks(radar_peaks, ir_peaks, window_s=args.match_window_s)
    if not matched:
        print('[SYNC] No matched events. Cannot compute drift.')
        sys.exit(2)

    deltas_ms = np.array([(ir - rad) * 1000.0 for rad, ir in matched])
    print('\n[SYNC] Matched events:')
    for (rad, ir), d in zip(matched, deltas_ms, strict=False):
        print(f'  radar t={rad:7.3f}s  ir t={ir:7.3f}s  dt={d:+7.1f} ms')

    abs_d = np.abs(deltas_ms)
    print('\n[SYNC] Drift summary:')
    print(f'  events matched   : {len(matched)}')
    print(f'  mean |dt|         : {abs_d.mean():6.1f} ms')
    print(f'  max  |dt|         : {abs_d.max():6.1f} ms')
    print(f'  std  dt           : {deltas_ms.std():6.1f} ms')

    ok = abs_d.max() <= args.max_drift_ms
    verdict = 'PASS' if ok else 'FAIL'
    print(f'\n[SYNC] Verdict (max |dt| <= {args.max_drift_ms:.0f} ms): {verdict}')

    # Update manifest
    try:
        m = Manifest.load(args.session_dir)
        m.sync.max_drift_ms_observed = float(abs_d.max())
        m.sync.note = (m.sync.note +
                       f' [check_sync: {len(matched)} events, '
                       f'max |dt|={abs_d.max():.1f}ms]').strip()
        m.write(args.session_dir)
        print('[SYNC] manifest.json updated with max_drift_ms_observed.')
    except Exception as e:  # noqa: BLE001
        print(f'[SYNC] Could not update manifest: {e}')

    if args.plot:
        import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
        ax1.plot(radar_t, radar_sig, color='#2c3e50', lw=0.8)
        ax1.scatter(radar_peaks, np.interp(radar_peaks, radar_t, radar_sig),
                    color='red', zorder=3, label='radar peaks')
        ax1.set_ylabel('|vz| (m/s)')
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.3)
        ax2.plot(ir_t, ir_sig, color='#e67e22', lw=0.8)
        ax2.scatter(ir_peaks, np.interp(ir_peaks, ir_t, ir_sig),
                    color='red', zorder=3, label='IR peaks')
        ax2.set_xlabel('time (s)')
        ax2.set_ylabel('mean |dt T| (°C/frame)')
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
