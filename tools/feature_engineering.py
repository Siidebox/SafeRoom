#!/usr/bin/env python3
"""
feature_engineering.py — Sliding-window feature extraction for fall detection ML.

Reads ML session CSVs (produced by MlCsvLogger), applies a sliding window over
each track's timeseries, and computes ~30 hand-crafted features per window.

Outputs
-------
For XGBoost / Random Forest (tabular features):
    X       : np.ndarray  (n_windows, n_features)
    y       : np.ndarray  (n_windows,)  — 1=fall, 0=not-fall
    groups  : np.ndarray  (n_windows,)  — session ID for LOSO-CV
    feature_names : list[str]

For LSTM / 1D-CNN (raw sequences):
    X_seq   : np.ndarray  (n_windows, window_size, n_seq_features)
    y       : np.ndarray  (n_windows,)
    groups  : np.ndarray  (n_windows,)
    seq_feature_names : list[str]

Usage:
    from feature_engineering import load_and_extract

    X, X_seq, y, groups, feat_names, seq_names = load_and_extract(
        csv_paths=['logs/ml_session_01.csv', 'logs/ml_session_02.csv'],
        window_size=30, stride=5
    )
"""

import os
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats as scipy_stats


# ── Configuration ────────────────────────────────────────────────────────────

WINDOW_SIZE = 30   # frames  (1.5 s at 20 fps)
STRIDE      = 5    # frames  (0.25 s)

# Raw per-frame features fed to the LSTM/CNN
SEQ_FEATURE_COLS = [
    'maxZ', 'height_m', 'vz', 'vx', 'vy', 'az',
    'pc_n_points', 'pc_mean_doppler', 'pc_std_doppler', 'pc_z_span',
]

# Label used when column is missing (old CSVs without 'label' column fall back
# to 'fall_detected' which is rule-based, not ground truth)
FALL_LABEL = 'fall'


# ── Per-window feature extraction ────────────────────────────────────────────

def extract_features_from_window(win: pd.DataFrame) -> dict:
    """
    Compute all features from a single window (DataFrame with window_size rows).

    Returns a flat dict {feature_name: float}.
    All values are finite floats (NaN replaced by 0).
    """
    feat = {}

    def safe(arr):
        """Convert to float array, replace NaN/inf with 0."""
        a = np.asarray(arr, dtype=np.float32)
        a = np.where(np.isfinite(a), a, 0.0)
        return a

    # ── Height features ──────────────────────────────────────────────────────
    maxZ     = safe(win['maxZ']) if 'maxZ' in win else np.zeros(len(win))
    height_m = safe(win['height_m']) if 'height_m' in win else np.zeros(len(win))

    feat['maxZ_start']  = float(maxZ[0])
    feat['maxZ_end']    = float(maxZ[-1])
    feat['maxZ_delta']  = float(maxZ[-1] - maxZ[0])
    feat['maxZ_ratio']  = float(maxZ[-1] / (maxZ[0] + 1e-6))
    feat['maxZ_min']    = float(maxZ.min())
    feat['maxZ_mean']   = float(maxZ.mean())
    feat['maxZ_std']    = float(maxZ.std())

    # Linear regression slope of maxZ over time
    if len(maxZ) > 1:
        t = np.arange(len(maxZ), dtype=np.float32)
        slope, _, _, _, _ = scipy_stats.linregress(t, maxZ)
        feat['maxZ_slope'] = float(slope)
    else:
        feat['maxZ_slope'] = 0.0

    feat['height_m_mean'] = float(height_m.mean())
    feat['height_m_std']  = float(height_m.std())

    # ── Velocity features ────────────────────────────────────────────────────
    vz = safe(win['vz']) if 'vz' in win else np.zeros(len(win))
    vx = safe(win['vx']) if 'vx' in win else np.zeros(len(win))
    vy = safe(win['vy']) if 'vy' in win else np.zeros(len(win))

    feat['vz_min']            = float(vz.min())
    feat['vz_mean']           = float(vz.mean())
    feat['vz_std']            = float(vz.std())
    feat['vz_integral']       = float(vz.sum())

    speed_3d   = np.sqrt(vx**2 + vy**2 + vz**2)
    speed_horiz = np.sqrt(vx**2 + vy**2)
    feat['speed_mean']         = float(speed_3d.mean())
    feat['speed_max']          = float(speed_3d.max())
    feat['v_horizontal_mean']  = float(speed_horiz.mean())

    # ── Acceleration features ────────────────────────────────────────────────
    az = safe(win['az']) if 'az' in win else np.zeros(len(win))
    ax = safe(win['ax']) if 'ax' in win else np.zeros(len(win))
    ay = safe(win['ay']) if 'ay' in win else np.zeros(len(win))

    feat['az_min']               = float(az.min())
    feat['az_max']               = float(az.max())
    feat['az_mean']              = float(az.mean())
    feat['az_std']               = float(az.std())
    accel_mag = np.sqrt(ax**2 + ay**2 + az**2)
    feat['accel_magnitude_max']  = float(accel_mag.max())
    feat['accel_magnitude_mean'] = float(accel_mag.mean())

    # ── Point cloud features ─────────────────────────────────────────────────
    n_pts    = safe(win['pc_n_points'])   if 'pc_n_points'    in win else np.zeros(len(win))
    dop_mean = safe(win['pc_mean_doppler']) if 'pc_mean_doppler' in win else np.zeros(len(win))
    dop_std  = safe(win['pc_std_doppler'])  if 'pc_std_doppler'  in win else np.zeros(len(win))
    snr_mean = safe(win['pc_mean_snr'])     if 'pc_mean_snr'     in win else np.zeros(len(win))
    z_span   = safe(win['pc_z_span'])       if 'pc_z_span'       in win else np.zeros(len(win))

    feat['pc_n_points_mean']        = float(n_pts.mean())
    feat['pc_n_points_std']         = float(n_pts.std())
    feat['pc_snr_mean']             = float(snr_mean.mean())
    feat['pc_z_span_mean']          = float(z_span.mean())
    feat['pc_z_span_std']           = float(z_span.std())
    feat['pc_doppler_spread_mean']  = float(dop_std.mean())
    feat['pc_mean_doppler_mean']    = float(dop_mean.mean())

    # ── Tracker metadata ─────────────────────────────────────────────────────
    conf = safe(win['confidence']) if 'confidence' in win else np.ones(len(win))
    g    = safe(win['g'])          if 'g'          in win else np.ones(len(win))

    feat['confidence_mean'] = float(conf.mean())
    feat['confidence_min']  = float(conf.min())
    feat['g_mean']          = float(g.mean())

    # ── Temporal shape features ──────────────────────────────────────────────
    # vz zero crossings (oscillation indicator)
    if len(vz) > 1:
        feat['vz_zero_crossings'] = float(np.sum(np.diff(np.sign(vz)) != 0))
    else:
        feat['vz_zero_crossings'] = 0.0

    # maxZ trend: sign of linear slope
    feat['maxZ_trend'] = float(np.sign(feat['maxZ_slope']))

    # Height at beginning vs end ratio (similar to rule-based FALL_RATIO)
    feat['height_drop_ratio'] = float(
        (feat['maxZ_start'] - feat['maxZ_end']) / (feat['maxZ_start'] + 1e-6)
    )

    return feat


def _window_label(win: pd.DataFrame) -> int:
    """
    Assign a binary label to a window.
    Returns 1 if any row is labeled 'fall', else 0.

    Falls back to 'fall_detected' column if 'label' column is absent
    (old CSVs with rule-based labels only — not ground truth).
    """
    if 'label' in win.columns:
        return 1 if (win['label'] == FALL_LABEL).any() else 0
    elif 'fall_detected' in win.columns:
        return 1 if (win['fall_detected'] == 1).any() else 0
    return 0


# ── Per-session extraction ───────────────────────────────────────────────────

def _extract_from_df(df: pd.DataFrame, session_id: str,
                     window_size: int, stride: int):
    """
    Extract windows from a single session DataFrame (one track per run).
    Returns lists: features_dicts, seq_arrays, labels, group_ids.
    """
    feats, seqs, labels, groups = [], [], [], []

    # Filter to rows that have a valid track (tid not empty)
    df_track = df[df['tid'].notna() & (df['tid'] != '')].copy()
    if df_track.empty:
        return feats, seqs, labels, groups

    # Convert numeric columns
    for col in df_track.columns:
        try:
            df_track[col] = pd.to_numeric(df_track[col], errors='ignore')
        except Exception:
            pass

    # Process each track independently (track IDs may change across sessions)
    for tid, track_df in df_track.groupby('tid'):
        track_df = track_df.sort_values('frameNum' if 'frameNum' in track_df.columns
                                         else track_df.index.name or track_df.columns[0])
        track_df = track_df.reset_index(drop=True)

        n = len(track_df)
        if n < window_size:
            continue

        for start in range(0, n - window_size + 1, stride):
            win = track_df.iloc[start: start + window_size]

            feat_dict = extract_features_from_window(win)
            feats.append(feat_dict)

            # Raw sequence for LSTM
            seq_cols_present = [c for c in SEQ_FEATURE_COLS if c in win.columns]
            seq_arr = win[seq_cols_present].values.astype(np.float32)
            # Pad missing columns with zeros
            if len(seq_cols_present) < len(SEQ_FEATURE_COLS):
                n_missing = len(SEQ_FEATURE_COLS) - len(seq_cols_present)
                pad = np.zeros((window_size, n_missing), dtype=np.float32)
                seq_arr = np.hstack([seq_arr, pad])
            # Replace NaN
            seq_arr = np.where(np.isfinite(seq_arr), seq_arr, 0.0)
            seqs.append(seq_arr)

            labels.append(_window_label(win))
            groups.append(session_id)

    return feats, seqs, labels, groups


# ── Public API ───────────────────────────────────────────────────────────────

def load_and_extract(csv_paths, window_size: int = WINDOW_SIZE,
                     stride: int = STRIDE):
    """
    Load all session CSVs and extract windowed features.

    Parameters
    ----------
    csv_paths : list[str] or list[Path]
        Paths to ML session CSV files (produced by MlCsvLogger).
    window_size : int
        Window length in frames (default 30 = 1.5 s at 20 fps).
    stride : int
        Stride between consecutive windows in frames (default 5 = 0.25 s).

    Returns
    -------
    X           : np.ndarray  (n_windows, n_features) — tabular feature matrix
    X_seq       : np.ndarray  (n_windows, window_size, n_seq_features) — raw sequences
    y           : np.ndarray  (n_windows,)  — binary labels (1=fall, 0=normal)
    groups      : np.ndarray  (n_windows,)  — session IDs (str) for LOSO-CV
    feat_names  : list[str]   — column names for X
    seq_names   : list[str]   — channel names for X_seq
    """
    all_feats, all_seqs, all_labels, all_groups = [], [], [], []

    for path in csv_paths:
        path = str(path)
        if not os.path.isfile(path):
            warnings.warn(f'CSV not found, skipping: {path}')
            continue

        try:
            df = pd.read_csv(path, low_memory=False)
        except Exception as e:
            warnings.warn(f'Failed to read {path}: {e}')
            continue

        # Derive session_id from filename stem
        session_id = Path(path).stem

        # Override with session_id column if present
        if 'session_id' in df.columns:
            ids = df['session_id'].dropna().unique()
            if len(ids) == 1 and str(ids[0]).strip():
                session_id = str(ids[0])

        feats, seqs, labels, groups = _extract_from_df(
            df, session_id, window_size, stride
        )
        all_feats.extend(feats)
        all_seqs.extend(seqs)
        all_labels.extend(labels)
        all_groups.extend(groups)

    if not all_feats:
        raise ValueError(
            'No windows extracted. Check that CSV files exist, have a valid '
            '"label" column, and contain enough frames per track.'
        )

    # Build consistent feature matrix (keys come from the first window dict)
    feat_names = list(all_feats[0].keys())
    X = np.array(
        [[f.get(k, 0.0) for k in feat_names] for f in all_feats],
        dtype=np.float32
    )

    X_seq  = np.stack(all_seqs, axis=0).astype(np.float32)   # (N, W, C)
    y      = np.array(all_labels, dtype=np.int32)
    groups = np.array(all_groups, dtype=object)

    n_falls = int(y.sum())
    print(f'Extracted {len(y)} windows from {len(csv_paths)} sessions  '
          f'({n_falls} fall, {len(y) - n_falls} normal)  '
          f'— {len(feat_names)} tabular features, {X_seq.shape[2]} sequence channels')

    return X, X_seq, y, groups, feat_names, SEQ_FEATURE_COLS


# ── CLI helper ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print('Usage: python feature_engineering.py logs/ml_*.csv')
        sys.exit(1)

    paths = sys.argv[1:]
    X, X_seq, y, groups, feat_names, seq_names = load_and_extract(paths)
    print(f'\nX shape:     {X.shape}')
    print(f'X_seq shape: {X_seq.shape}')
    print(f'y shape:     {y.shape}')
    print(f'Groups:      {sorted(set(groups))}')
    print(f'\nFeature names ({len(feat_names)}):')
    for i, n in enumerate(feat_names):
        print(f'  {i:2d}  {n}')
