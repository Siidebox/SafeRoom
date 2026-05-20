#!/usr/bin/env python3
"""explore_ir_kinetics.py — Phase 1 of A_kinetic (design 2026-05-19).

Loads labeled session directories (radar.csv + thermal.npz + manifest.json)
and produces per-class distributions and ROC plots for candidate kinetic
features computed from the IR stream. The output decides whether to spec a
standalone IR kinetic detector.

Features computed per labeled event window (-1s..+1s around event start):
  - centroid_vy_peak   : peak |dy/dt| of blob centroid (px/s)
  - bbox_height_drop   : max drop in bbox height over any 250ms window (px)
  - aspect_change_peak : peak |d(h/w)/dt| (px-ratio/s)
  - temporal_grad_peak : per-frame max |frame[t] - frame[t-1]| (°C)

Decision criterion (spec): if any feature has ROC AUC ≥ 0.7 on a held-out
session, advance to phase 2 (separate spec for IrKineticDetector).

Usage:
  python tools/explore_ir_kinetics.py sessions/  [--out figures/ir_kinetics/]
"""
import argparse
import json
import os
import sys
import warnings
from pathlib import Path

import matplotlib
matplotlib.use('Agg')                # non-interactive: works headless on Pi
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.metrics import roc_auc_score, roc_curve

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from manifest_schema import Manifest


FALL_LABELS = {'fall', 'fall_fast'}
WINDOW_S    = 2.0     # 1 s before + 1 s after label start


# ── feature extraction ───────────────────────────────────────────────────────

def _blob_mask(frame, bg_mean, bg_std, k=3.0):
    return frame > (bg_mean + k * bg_std)


def _per_frame_quantities(frames, bg_mean, bg_std):
    cys, cxs, hs, ws = [], [], [], []
    for f in frames:
        m = _blob_mask(f, bg_mean, bg_std)
        ys, xs = np.nonzero(m)
        if ys.size == 0:
            cys.append(np.nan); cxs.append(np.nan)
            hs.append(np.nan); ws.append(np.nan)
            continue
        cys.append(float(ys.mean()))
        cxs.append(float(xs.mean()))
        hs.append(float(ys.max() - ys.min() + 1))
        ws.append(float(xs.max() - xs.min() + 1))
    return (np.asarray(cys), np.asarray(cxs),
            np.asarray(hs), np.asarray(ws))


def extract_features(thermal_npz_path: str,
                     window_ts: tuple[int, int]) -> dict:
    d = np.load(thermal_npz_path)
    frames = d['frames']
    if 't_mono_ns' not in d.files:
        warnings.warn('thermal.npz missing t_mono_ns; skipping')
        return {}
    t_mono = d['t_mono_ns'].astype(np.int64)

    start_ns, end_ns = window_ts
    mask = (t_mono >= start_ns) & (t_mono <= end_ns)
    if mask.sum() < 4:
        return {}
    frames_w = frames[mask]
    ts_w = t_mono[mask]

    # Cheap per-event background = median of first 4 frames in the window
    bg_mean = np.median(frames_w[:4], axis=0)
    bg_std = np.std(frames_w[:4], axis=0).clip(min=0.1)

    cy, cx, h, w = _per_frame_quantities(frames_w, bg_mean, bg_std)
    dt_s = np.diff(ts_w) / 1e9
    dt_s[dt_s <= 0] = 1e-3

    vy = np.diff(cy) / dt_s
    centroid_vy_peak = float(np.nanmax(np.abs(vy))) if vy.size else 0.0

    # bbox height drop over 250ms windows
    bbox_drop = 0.0
    for i in range(len(h)):
        for j in range(i + 1, len(h)):
            if (ts_w[j] - ts_w[i]) / 1e9 > 0.250:
                break
            drop = float(h[i] - h[j])
            if drop > bbox_drop and not np.isnan(drop):
                bbox_drop = drop

    aspect = h / np.maximum(w, 1.0)
    aspect_d = np.diff(aspect) / dt_s
    aspect_change_peak = float(np.nanmax(np.abs(aspect_d))) if aspect_d.size else 0.0

    grad_peak = float(np.max(np.abs(np.diff(frames_w.astype(np.float32),
                                            axis=0)).reshape(-1)))

    return {
        'centroid_vy_peak':   centroid_vy_peak,
        'bbox_height_drop':   bbox_drop,
        'aspect_change_peak': aspect_change_peak,
        'temporal_grad_peak': grad_peak,
    }


# ── per-session driver ───────────────────────────────────────────────────────

def collect_events(session_dir: str) -> list[dict]:
    manifest_path = os.path.join(session_dir, 'manifest.json')
    npz_path      = os.path.join(session_dir, 'thermal.npz')
    if not os.path.isfile(manifest_path) or not os.path.isfile(npz_path):
        return []

    m = Manifest.load(session_dir)
    out: list[dict] = []
    for span in m.labels:
        label = span.label
        if label not in FALL_LABELS and label not in {'walk', 'stand', 'sit', 'lie', 'none'}:
            continue
        start_ns = int(span.t_start_mono_ns) - int(0.5 * 1e9)
        end_ns   = int(span.t_start_mono_ns) + int(WINDOW_S * 1e9)
        feats = extract_features(npz_path, (start_ns, end_ns))
        if not feats:
            continue
        out.append({
            'session': os.path.basename(session_dir),
            'label': 'fall' if label in FALL_LABELS else 'non_fall',
            **feats,
        })
    return out


# ── main ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('sessions_dir', help='Directory containing session subdirs')
    ap.add_argument('--out', default='figures/ir_kinetics',
                    help='Output directory for plots and CSV (default: figures/ir_kinetics)')
    args = ap.parse_args()

    rows: list[dict] = []
    for child in sorted(Path(args.sessions_dir).iterdir()):
        if child.is_dir():
            rows.extend(collect_events(str(child)))

    if not rows:
        sys.exit('No labeled fall/non_fall events found.')

    df = pd.DataFrame(rows)
    os.makedirs(args.out, exist_ok=True)
    df.to_csv(os.path.join(args.out, 'features.csv'), index=False)

    feature_cols = [c for c in df.columns
                    if c not in ('session', 'label')]
    y = (df['label'] == 'fall').astype(int).to_numpy()

    print('event counts:', dict(df['label'].value_counts()))
    print('feature ROC AUC:')

    fig, axes = plt.subplots(1, len(feature_cols), figsize=(4 * len(feature_cols), 4))
    if len(feature_cols) == 1:
        axes = [axes]

    decision_pass = False
    for ax, col in zip(axes, feature_cols):
        x = df[col].to_numpy()
        x = np.where(np.isfinite(x), x, 0.0)
        try:
            auc = roc_auc_score(y, x)
        except ValueError:
            auc = float('nan')
        fpr, tpr, _ = roc_curve(y, x)
        ax.plot(fpr, tpr, lw=2)
        ax.plot([0, 1], [0, 1], 'k--', lw=0.5)
        ax.set_title(f'{col}\nAUC = {auc:.3f}')
        ax.set_xlabel('FPR')
        ax.set_ylabel('TPR')
        print(f'  {col:22s} AUC = {auc:.3f}')
        if auc >= 0.7:
            decision_pass = True

    plt.tight_layout()
    plt.savefig(os.path.join(args.out, 'roc.png'), dpi=120)
    plt.close()

    print(f'\nDecision criterion (spec 2026-05-19): '
          f'{"PHASE-2 (spec IrKineticDetector)" if decision_pass else "STOP — kinetic not viable"}')
    sys.exit(0 if decision_pass else 0)   # exit 0 either way; criterion is informational


if __name__ == '__main__':
    main()
