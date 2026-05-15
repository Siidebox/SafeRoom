#!/usr/bin/env python3
"""
migrate_legacy_csv.py — Convert legacy SafeRoom session CSVs to MlCsvLogger
schema for the ML pipeline smoke test.

Legacy CSVs (old `CsvLogger`) lack 17 columns required by `feature_engineering`
and the downstream tools. This script fills the gap with sensible defaults:

  - session_id        = filename stem
  - ax, ay, az        = finite-diff of vx, vy, vz per (session, tid)
  - g, confidence     = 1.0 (neutral defaults matching feature_engineering fallbacks)
  - pc_*              = 0.0 (no point cloud info in legacy logs)
  - num_total_points  = 0
  - num_tracks        = 0
  - label             = 'fall' if fall_detected==1 else 'normal' (rule-based proxy)

For smoke-test purposes only. The synthetic `label` column derives from the
rule-based detector, so models trained on this leak the rule into the target.
DO NOT use migrated files for reported metrics — only for pipeline shake-out.

Usage:
    python tools/migrate_legacy_csv.py logs/ logs_migrated/
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Same target schema as ml_logger.ML_COLUMNS
ML_COLUMNS = [
    'timestamp', 'session_id', 'frameNum', 'presence',
    'tid', 'x', 'y', 'z', 'vx', 'vy', 'vz',
    'ax', 'ay', 'az', 'g', 'confidence',
    'height_m', 'maxZ', 'minZ', 'maxZ_ref', 'peak_vz',
    'fall_detected', 'faint_detected',
    'pc_n_points',
    'pc_mean_doppler', 'pc_std_doppler',
    'pc_mean_snr',     'pc_std_snr',
    'pc_mean_z',       'pc_std_z',
    'pc_z_span',
    'num_total_points', 'num_tracks',
    'label',
]


def _finite_diff_accel(df: pd.DataFrame) -> pd.DataFrame:
    """Add ax/ay/az columns as finite-diff of vx/vy/vz, grouped by tid."""
    df = df.copy()
    # ensure numeric
    for c in ('vx', 'vy', 'vz', 'timestamp', 'tid'):
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors='coerce')

    df['ax'] = 0.0
    df['ay'] = 0.0
    df['az'] = 0.0

    if 'tid' not in df.columns:
        return df

    for tid, idx in df.groupby('tid').groups.items():
        sub = df.loc[idx].sort_values('timestamp') if 'timestamp' in df.columns \
              else df.loc[idx]
        dt  = sub['timestamp'].diff().replace(0.0, np.nan) \
              if 'timestamp' in sub else pd.Series(0.05, index=sub.index)
        for vcol, acol in (('vx', 'ax'), ('vy', 'ay'), ('vz', 'az')):
            if vcol in sub.columns:
                dv = sub[vcol].diff()
                a  = (dv / dt).fillna(0.0).replace([np.inf, -np.inf], 0.0)
                df.loc[a.index, acol] = a.values
    return df


def migrate(src_path: Path, dst_path: Path) -> str:
    """Migrate one CSV. Returns a short status string."""
    try:
        df = pd.read_csv(src_path, low_memory=False)
    except Exception as e:
        return f'FAIL read: {e}'

    if df.empty:
        # write empty file with full header for downstream tools
        pd.DataFrame(columns=ML_COLUMNS).to_csv(dst_path, index=False)
        return 'empty (header only)'

    have = set(df.columns)

    # If already has the full schema, just copy unchanged.
    if all(c in have for c in ML_COLUMNS):
        df[ML_COLUMNS].to_csv(dst_path, index=False)
        return 'already MlCsvLogger schema'

    # Derive missing columns
    if 'ax' not in have or 'ay' not in have or 'az' not in have:
        df = _finite_diff_accel(df)

    if 'session_id' not in have:
        df['session_id'] = src_path.stem
    if 'g' not in have:
        df['g'] = 1.0
    if 'confidence' not in have:
        df['confidence'] = 1.0
    if 'faint_detected' not in have:
        df['faint_detected'] = 0
    for pc_col in ['pc_n_points', 'pc_mean_doppler', 'pc_std_doppler',
                   'pc_mean_snr', 'pc_std_snr', 'pc_mean_z', 'pc_std_z',
                   'pc_z_span']:
        if pc_col not in df.columns:
            df[pc_col] = 0.0
    if 'num_total_points' not in df.columns:
        df['num_total_points'] = 0
    if 'num_tracks' not in df.columns:
        df['num_tracks'] = 0

    if 'label' not in df.columns:
        if 'fall_detected' in df.columns:
            fd = pd.to_numeric(df['fall_detected'], errors='coerce').fillna(0)
            df['label'] = np.where(fd == 1, 'fall', 'normal')
        else:
            df['label'] = 'normal'

    # Ensure every target column exists before reordering
    for c in ML_COLUMNS:
        if c not in df.columns:
            df[c] = ''

    df[ML_COLUMNS].to_csv(dst_path, index=False)
    return f'migrated ({len(df)} rows)'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src_dir', type=Path)
    ap.add_argument('dst_dir', type=Path)
    args = ap.parse_args()

    args.dst_dir.mkdir(parents=True, exist_ok=True)
    csvs = sorted(args.src_dir.glob('*.csv'))
    if not csvs:
        print(f'No CSVs in {args.src_dir}', file=sys.stderr)
        sys.exit(1)

    print(f'Migrating {len(csvs)} CSVs from {args.src_dir} -> {args.dst_dir}')
    for src in csvs:
        dst = args.dst_dir / src.name
        status = migrate(src, dst)
        print(f'  {src.name:<48s} {status}')


if __name__ == '__main__':
    main()
