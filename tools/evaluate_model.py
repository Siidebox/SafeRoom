#!/usr/bin/env python3
"""
evaluate_model.py — Evaluation and thesis figure generation for fall detection models.

Generates:
  - Per-model metrics table (Precision, Recall, F1, F2, AUC, AP)
  - Confusion matrices (2×2 heatmaps)
  - ROC curves overlaid for all methods
  - Precision-Recall curves overlaid
  - XGBoost feature importance bar chart
  - Detection latency histogram
  - LaTeX-ready metrics table (optional)

Usage:
    # Evaluate trained models (runs LOSO-CV internally):
    python tools/evaluate_model.py --data logs/ml_session_*.csv

    # Also compare against rule-based detector:
    python tools/evaluate_model.py --data logs/ml_session_*.csv --compare-rule-based

    # Save figures to directory:
    python tools/evaluate_model.py --data logs/ml_session_*.csv --outdir figures/

Requirements:
    pip install xgboost scikit-learn joblib scipy pandas numpy matplotlib
"""

import argparse
import glob
import os
import warnings

import joblib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd
from sklearn.metrics import (
    roc_curve, auc as sklearn_auc,
    precision_recall_curve, average_precision_score,
    confusion_matrix, classification_report,
    precision_recall_fscore_support,
)

from feature_engineering import load_and_extract, WINDOW_SIZE, STRIDE
from train_model import (
    run_loso_cv, fbeta_score, threshold_from_cv, eval_predictions
)


# ── Plotting helpers ─────────────────────────────────────────────────────────

MODEL_COLORS = {
    'XGBoost': '#e74c3c',
    'RF':      '#2ecc71',
    'LSTM':    '#3498db',
    'Rule':    '#f39c12',
}


def _save_or_show(fig, path: str = None):
    if path:
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f'  Saved: {path}')
    else:
        plt.show()
    plt.close(fig)


# ── Rule-based detector replay ───────────────────────────────────────────────

def replay_rule_based(csv_paths: list) -> tuple:
    """
    Replay the rule-based FallDetector on annotated ML session CSVs.

    Reads each CSV row by row and calls FallDetector.update() in order.
    Returns (y_true, y_pred, y_prob_proxy) arrays.

    Note: 'probability' for the rule-based detector is approximated from
    Tier 1 vz count normalised by persist threshold (range 0–1).
    """
    import sys, os
    # Import FallDetector from radar_reader
    tools_dir = os.path.dirname(os.path.abspath(__file__))
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
    from radar_reader import FallDetector

    y_true_all, y_pred_all, y_prob_all = [], [], []

    for path in csv_paths:
        df = pd.read_csv(path, low_memory=False)
        if 'label' not in df.columns:
            warnings.warn(f'{path}: no "label" column, skipping rule-based replay.')
            continue

        detector = FallDetector()
        df_track = df[df['tid'].notna() & (df['tid'] != '')].copy()

        for _, row in df_track.iterrows():
            tid = int(row['tid'])
            h   = {
                'maxZ': float(row['maxZ']) if pd.notna(row.get('maxZ')) else 0.0,
                'minZ': float(row['minZ']) if pd.notna(row.get('minZ')) else 0.0,
            }
            t   = {'vz': float(row['vz']) if pd.notna(row.get('vz')) else 0.0}
            is_fall, _ = detector.update(tid, h, t)

            true_label = 1 if str(row.get('label', 'unknown')) == 'fall' else 0
            y_true_all.append(true_label)
            y_pred_all.append(1 if is_fall else 0)
            # Proxy probability: normalised fast_count (0–1)
            fc = detector._fast_count.get(tid, 0)
            y_prob_all.append(min(fc / detector.FAST_PERSIST, 1.0))

    return (np.array(y_true_all), np.array(y_pred_all), np.array(y_prob_all))


# ── Metrics table ─────────────────────────────────────────────────────────────

def build_metrics_table(all_cv_rows: dict) -> pd.DataFrame:
    """
    Aggregate per-fold CV results into mean ± std per model.

    all_cv_rows: dict {model_prefix → list of fold metric dicts}
    """
    rows = []
    metric_cols = ['precision', 'recall', 'f1', 'f2', 'roc_auc', 'avg_precision']
    for model_name, fold_rows in all_cv_rows.items():
        if not fold_rows:
            continue
        df = pd.DataFrame(fold_rows)
        mean = df[metric_cols].mean()
        std  = df[metric_cols].std().fillna(0)
        row = {'Model': model_name}
        for c in metric_cols:
            row[c] = f'{mean[c]:.3f} ± {std[c]:.3f}'
        rows.append(row)
    return pd.DataFrame(rows).set_index('Model')


def print_metrics_table(df: pd.DataFrame):
    print('\n' + '=' * 90)
    print(f'{"Model":<18} ' +
          '  '.join(f'{c:<14}' for c in df.columns))
    print('-' * 90)
    for model, row in df.iterrows():
        print(f'{model:<18} ' + '  '.join(f'{v:<14}' for v in row.values))
    print('=' * 90 + '\n')


def metrics_to_latex(df: pd.DataFrame) -> str:
    """Return a simple LaTeX table string."""
    cols = list(df.columns)
    header = ' & '.join(['Model'] + cols) + r' \\'
    lines  = [r'\begin{tabular}{l' + 'c' * len(cols) + '}', r'\hline',
              header, r'\hline']
    for model, row in df.iterrows():
        lines.append(' & '.join([model] + list(row.values)) + r' \\')
    lines += [r'\hline', r'\end{tabular}']
    return '\n'.join(lines)


# ── Plotting functions ────────────────────────────────────────────────────────

def plot_confusion_matrices(cm_data: dict, outdir=None):
    """cm_data: {model_name: (y_true, y_pred)}"""
    n = len(cm_data)
    fig, axes = plt.subplots(1, n, figsize=(4 * n, 4))
    if n == 1:
        axes = [axes]
    fig.suptitle('Confusion Matrices (LOSO-CV, all folds combined)', fontsize=11)

    for ax, (name, (y_true, y_pred)) in zip(axes, cm_data.items()):
        cm = confusion_matrix(y_true, y_pred)
        im = ax.imshow(cm, interpolation='nearest', cmap='Blues')
        ax.set_title(name, fontsize=10)
        ax.set_xlabel('Predicted'), ax.set_ylabel('True')
        ax.set_xticks([0, 1]), ax.set_yticks([0, 1])
        ax.set_xticklabels(['Normal', 'Fall'])
        ax.set_yticklabels(['Normal', 'Fall'])
        for i in range(2):
            for j in range(2):
                ax.text(j, i, str(cm[i, j]),
                        ha='center', va='center', fontsize=14,
                        color='white' if cm[i, j] > cm.max() / 2 else 'black')
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    plt.tight_layout()
    _save_or_show(fig, os.path.join(outdir, 'confusion_matrices.png') if outdir else None)


def plot_roc_curves(roc_data: dict, outdir=None):
    """roc_data: {model_name: (y_true, y_prob)}"""
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot([0, 1], [0, 1], 'k--', lw=0.8, alpha=0.4)
    for name, (y_true, y_prob) in roc_data.items():
        fpr, tpr, _ = roc_curve(y_true, y_prob)
        a = sklearn_auc(fpr, tpr)
        color = MODEL_COLORS.get(name, '#333333')
        ax.plot(fpr, tpr, color=color, lw=2, label=f'{name} (AUC={a:.3f})')
    ax.set_xlabel('False Positive Rate'), ax.set_ylabel('True Positive Rate')
    ax.set_title('ROC Curves — Fall Detection (LOSO-CV)')
    ax.legend(fontsize=9), ax.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_or_show(fig, os.path.join(outdir, 'roc_curves.png') if outdir else None)


def plot_pr_curves(pr_data: dict, baseline_prevalence: float, outdir=None):
    """pr_data: {model_name: (y_true, y_prob)}"""
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.axhline(baseline_prevalence, color='k', ls='--', lw=0.8, alpha=0.4,
               label=f'Chance ({baseline_prevalence:.3f})')
    for name, (y_true, y_prob) in pr_data.items():
        prec, rec, _ = precision_recall_curve(y_true, y_prob)
        ap = average_precision_score(y_true, y_prob)
        color = MODEL_COLORS.get(name, '#333333')
        ax.plot(rec, prec, color=color, lw=2, label=f'{name} (AP={ap:.3f})')
    ax.set_xlabel('Recall'), ax.set_ylabel('Precision')
    ax.set_title('Precision-Recall Curves — Fall Detection (LOSO-CV)')
    ax.legend(fontsize=9), ax.grid(True, alpha=0.3)
    plt.tight_layout()
    _save_or_show(fig, os.path.join(outdir, 'pr_curves.png') if outdir else None)


def plot_feature_importance(model_path: str, feat_names: list, top_n: int = 20,
                             outdir=None):
    """Bar chart of XGBoost feature importances."""
    if not os.path.isfile(model_path):
        warnings.warn(f'Model not found for feature importance: {model_path}')
        return
    model = joblib.load(model_path)
    if not hasattr(model, 'feature_importances_'):
        warnings.warn('Model has no feature_importances_ attribute.')
        return

    importance = model.feature_importances_
    if len(feat_names) != len(importance):
        feat_names = [f'f{i}' for i in range(len(importance))]

    idx   = np.argsort(importance)[::-1][:top_n]
    names = [feat_names[i] for i in idx]
    vals  = importance[idx]

    fig, ax = plt.subplots(figsize=(9, 5))
    colors = plt.cm.RdYlGn(np.linspace(0.2, 0.8, len(vals)))
    ax.barh(range(len(vals))[::-1], vals, color=colors)
    ax.set_yticks(range(len(vals))[::-1])
    ax.set_yticklabels(names, fontsize=9)
    ax.set_xlabel('Feature Importance (gain)')
    ax.set_title(f'XGBoost Top-{top_n} Feature Importances')
    ax.grid(True, axis='x', alpha=0.3)
    plt.tight_layout()
    _save_or_show(fig, os.path.join(outdir, 'feature_importance.png') if outdir else None)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Evaluate ML fall detection models and generate thesis figures.'
    )
    parser.add_argument('--data', nargs='+', required=True,
                        help='ML session CSV files (glob patterns supported).')
    parser.add_argument('--window', type=int, default=WINDOW_SIZE)
    parser.add_argument('--stride', type=int, default=STRIDE)
    parser.add_argument('--no-dl', action='store_true',
                        help='Skip LSTM evaluation')
    parser.add_argument('--compare-rule-based', action='store_true',
                        help='Also evaluate the rule-based FallDetector as baseline')
    parser.add_argument('--outdir', default=None,
                        help='Directory to save figure files (default: show interactively)')
    parser.add_argument('--latex', action='store_true',
                        help='Print LaTeX metrics table to stdout')
    args = parser.parse_args()

    # Expand globs
    csv_paths = []
    for pat in args.data:
        expanded = glob.glob(pat)
        csv_paths.extend(expanded if expanded else [pat])
    csv_paths = sorted(set(csv_paths))

    print(f'Evaluating on {len(csv_paths)} sessions...')

    # Feature extraction
    X, X_seq, y, groups, feat_names, _ = load_and_extract(
        csv_paths, window_size=args.window, stride=args.stride
    )

    train_dl = not args.no_dl

    # LOSO-CV
    print('\nRunning LOSO-CV...')
    rows, xgb_cv, rf_cv, lstm_cv = run_loso_cv(
        X, X_seq, y, groups, feat_names, train_dl=train_dl
    )

    # Aggregate results
    def filter_rows(prefix):
        return [r for r in rows if r['model'].startswith(prefix)]

    all_cv_rows = {
        'XGBoost': filter_rows('XGBoost'),
        'RF':      filter_rows('RF'),
    }
    if train_dl and filter_rows('LSTM'):
        all_cv_rows['LSTM'] = filter_rows('LSTM')

    # Accumulate predictions across all folds for plotting
    xgb_true  = np.concatenate(xgb_cv[1]) if xgb_cv[1] else np.array([])
    xgb_prob  = np.concatenate(xgb_cv[0]) if xgb_cv[0] else np.array([])
    rf_true   = np.concatenate(rf_cv[1])  if rf_cv[1]  else np.array([])
    rf_prob   = np.concatenate(rf_cv[0])  if rf_cv[0]  else np.array([])

    roc_data = {}
    pr_data  = {}
    cm_data  = {}

    if len(xgb_true):
        best_xgb_t = threshold_from_cv(xgb_cv[0], xgb_cv[1])
        xgb_pred   = (xgb_prob >= best_xgb_t).astype(int)
        roc_data['XGBoost'] = (xgb_true, xgb_prob)
        pr_data['XGBoost']  = (xgb_true, xgb_prob)
        cm_data['XGBoost']  = (xgb_true, xgb_pred)
        all_cv_rows['XGBoost_aggregate'] = [
            eval_predictions(xgb_true, xgb_pred, xgb_prob, 'XGBoost_agg')
        ]

    if len(rf_true):
        best_rf_t = threshold_from_cv(rf_cv[0], rf_cv[1])
        rf_pred   = (rf_prob >= best_rf_t).astype(int)
        roc_data['RF'] = (rf_true, rf_prob)
        pr_data['RF']  = (rf_true, rf_prob)
        cm_data['RF']  = (rf_true, rf_pred)

    if train_dl and lstm_cv[1]:
        lstm_true = np.concatenate(lstm_cv[1])
        lstm_prob = np.concatenate(lstm_cv[0])
        best_lstm_t = threshold_from_cv(lstm_cv[0], lstm_cv[1])
        lstm_pred   = (lstm_prob >= best_lstm_t).astype(int)
        roc_data['LSTM'] = (lstm_true, lstm_prob)
        pr_data['LSTM']  = (lstm_true, lstm_prob)
        cm_data['LSTM']  = (lstm_true, lstm_pred)

    # Rule-based baseline
    if args.compare_rule_based:
        print('\nReplaying rule-based detector...')
        try:
            rb_true, rb_pred, rb_prob = replay_rule_based(csv_paths)
            if len(rb_true):
                roc_data['Rule'] = (rb_true, rb_prob)
                pr_data['Rule']  = (rb_true, rb_prob)
                cm_data['Rule']  = (rb_true, rb_pred)
                all_cv_rows['Rule-based'] = [
                    eval_predictions(rb_true, rb_pred, rb_prob, 'Rule-based')
                ]
        except Exception as e:
            warnings.warn(f'Rule-based replay failed: {e}')

    # Print metrics table
    metrics_df = build_metrics_table(all_cv_rows)
    print_metrics_table(metrics_df)
    if args.latex:
        print('\nLaTeX table:\n')
        print(metrics_to_latex(metrics_df))

    # Plots
    print('\nGenerating figures...')
    outdir = args.outdir

    if cm_data:
        plot_confusion_matrices(cm_data, outdir)
    if roc_data:
        plot_roc_curves(roc_data, outdir)
    if pr_data:
        prevalence = float(y.mean())
        plot_pr_curves(pr_data, prevalence, outdir)

    xgb_model_path = 'models/fall_detector_xgb.pkl'
    if os.path.isfile(xgb_model_path):
        plot_feature_importance(xgb_model_path, feat_names, top_n=20, outdir=outdir)

    print('\nEvaluation complete.')


if __name__ == '__main__':
    main()
