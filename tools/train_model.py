#!/usr/bin/env python3
"""
train_model.py — ML fall detection model training with Leave-One-Session-Out CV.

Trains three models and evaluates with LOSO-CV:
  1. XGBoost         (primary — interpretable, small-data robust)
  2. Random Forest   (classical baseline)
  3. LSTM / 1D-CNN   (deep learning comparison, requires torch)

Usage:
    python tools/train_model.py --data logs/ml_session_*.csv
    python tools/train_model.py --data logs/ml_session_*.csv --no-dl

Output:
    models/fall_detector_xgb.pkl   — XGBoost (best threshold from CV)
    models/fall_detector_rf.pkl    — Random Forest
    models/fall_detector_lstm.pt   — LSTM (if torch available)
    models/cv_results.csv          — Per-fold metrics table

Requirements:
    pip install xgboost scikit-learn joblib scipy pandas numpy
    pip install torch          (optional, for LSTM)
"""

import argparse
import glob
import json
import os
import warnings
from pathlib import Path

import joblib
import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import (
    classification_report, confusion_matrix,
    roc_auc_score, average_precision_score,
    precision_recall_fscore_support,
)
from sklearn.preprocessing import StandardScaler
import xgboost as xgb

from feature_engineering import load_and_extract, WINDOW_SIZE, STRIDE


# ── Helpers ───────────────────────────────────────────────────────────────────

def fbeta_score(precision, recall, beta=2.0):
    """F-beta score (default beta=2 weights recall higher than precision)."""
    if precision + recall == 0:
        return 0.0
    return (1 + beta**2) * precision * recall / (beta**2 * precision + recall)


def eval_predictions(y_true, y_pred, y_prob, name: str) -> dict:
    """Compute all metrics for one model on one fold."""
    p, r, f1, _ = precision_recall_fscore_support(y_true, y_pred, average='binary',
                                                    zero_division=0)
    f2 = fbeta_score(p, r, beta=2.0)
    cm = confusion_matrix(y_true, y_pred)
    tn, fp, fn, tp = cm.ravel() if cm.size == 4 else (0, 0, 0, 0)
    try:
        auc = roc_auc_score(y_true, y_prob)
    except ValueError:
        auc = float('nan')
    try:
        ap = average_precision_score(y_true, y_prob)
    except ValueError:
        ap = float('nan')
    return {
        'model': name,
        'precision': round(p, 4), 'recall': round(r, 4),
        'f1': round(f1, 4), 'f2': round(f2, 4),
        'roc_auc': round(auc, 4), 'avg_precision': round(ap, 4),
        'tp': int(tp), 'fp': int(fp), 'tn': int(tn), 'fn': int(fn),
    }


def threshold_from_cv(probs_list, labels_list, beta=2.0):
    """
    Find the probability threshold that maximises F-beta on concatenated CV folds.
    Returns the optimal threshold value.
    """
    probs  = np.concatenate(probs_list)
    labels = np.concatenate(labels_list)
    best_t, best_fb = 0.5, 0.0
    for t in np.arange(0.1, 0.91, 0.05):
        preds = (probs >= t).astype(int)
        p, r, f1, _ = precision_recall_fscore_support(
            labels, preds, average='binary', zero_division=0
        )
        fb = fbeta_score(p, r, beta)
        if fb > best_fb:
            best_fb = fb
            best_t  = t
    return float(best_t)


# ── LSTM / 1D-CNN definition ─────────────────────────────────────────────────

def _build_lstm(n_features: int, hidden: int = 64, dropout: float = 0.3):
    """Small LSTM classifier. Input: (batch, seq_len, n_features)."""
    import torch
    import torch.nn as nn

    class FallLSTM(nn.Module):
        def __init__(self):
            super().__init__()
            self.lstm  = nn.LSTM(n_features, hidden, batch_first=True,
                                  num_layers=1, dropout=0.0)
            self.drop  = nn.Dropout(dropout)
            self.fc    = nn.Linear(hidden, 1)

        def forward(self, x):
            _, (h, _) = self.lstm(x)
            h = h.squeeze(0)
            return self.fc(self.drop(h)).squeeze(-1)

    return FallLSTM()


def train_lstm_fold(X_train_seq, y_train, X_val_seq, y_val,
                    epochs: int = 30, batch_size: int = 32, lr: float = 1e-3):
    """Train one LSTM fold. Returns (model, val_probs)."""
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset

    n_features = X_train_seq.shape[2]
    model = _build_lstm(n_features)

    pos_weight = torch.tensor(
        [(y_train == 0).sum() / max((y_train == 1).sum(), 1)],
        dtype=torch.float32
    )
    criterion = nn.BCEWithLogitsLoss(pos_weight=pos_weight)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, patience=5, factor=0.5, verbose=False
    )

    X_t = torch.tensor(X_train_seq, dtype=torch.float32)
    y_t = torch.tensor(y_train,     dtype=torch.float32)
    ds  = TensorDataset(X_t, y_t)
    dl  = DataLoader(ds, batch_size=batch_size, shuffle=True)

    best_state, best_val_loss = None, float('inf')

    for epoch in range(epochs):
        model.train()
        for xb, yb in dl:
            optimizer.zero_grad()
            loss = criterion(model(xb), yb)
            loss.backward()
            optimizer.step()

        # Validation
        model.eval()
        with torch.no_grad():
            Xv = torch.tensor(X_val_seq, dtype=torch.float32)
            yv = torch.tensor(y_val,     dtype=torch.float32)
            val_loss = criterion(model(Xv), yv).item()

        scheduler.step(val_loss)
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            import copy
            best_state = copy.deepcopy(model.state_dict())

    model.load_state_dict(best_state)
    model.eval()
    with torch.no_grad():
        Xv  = torch.tensor(X_val_seq, dtype=torch.float32)
        probs = torch.sigmoid(model(Xv)).numpy()
    return model, probs


# ── LOSO-CV ───────────────────────────────────────────────────────────────────

def run_loso_cv(X, X_seq, y, groups, feat_names, train_dl=True):
    """
    Leave-One-Session-Out cross-validation.
    Returns per-fold metrics and accumulated (probs, labels) for threshold tuning.
    """
    unique_sessions = sorted(set(groups))
    n_sessions = len(unique_sessions)

    if n_sessions < 2:
        warnings.warn(
            f'Only {n_sessions} session(s) found. LOSO-CV requires at least 2. '
            'Results may be overly optimistic.'
        )

    all_rows  = []
    xgb_probs_all, rf_probs_all, lstm_probs_all = [], [], []
    xgb_labs, rf_labs, lstm_labs = [], [], []

    for held_out in unique_sessions:
        train_mask = groups != held_out
        val_mask   = groups == held_out

        if val_mask.sum() == 0 or train_mask.sum() == 0:
            continue

        X_tr, X_va   = X[train_mask],     X[val_mask]
        y_tr, y_va   = y[train_mask],     y[val_mask]
        Xs_tr, Xs_va = X_seq[train_mask], X_seq[val_mask]

        # ── XGBoost ──────────────────────────────────────────────────────────
        scale_pos = max((y_tr == 0).sum() / max((y_tr == 1).sum(), 1), 1.0)
        xgb_model = xgb.XGBClassifier(
            n_estimators=200, max_depth=5, learning_rate=0.1,
            scale_pos_weight=scale_pos,
            eval_metric='logloss', verbosity=0,
            use_label_encoder=False, random_state=42,
        )
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            xgb_model.fit(X_tr, y_tr)
        xgb_prob = xgb_model.predict_proba(X_va)[:, 1]
        xgb_pred = (xgb_prob >= 0.5).astype(int)
        all_rows.append(eval_predictions(y_va, xgb_pred, xgb_prob, f'XGBoost_fold_{held_out}'))
        xgb_probs_all.append(xgb_prob)
        xgb_labs.append(y_va)

        # ── Random Forest ────────────────────────────────────────────────────
        rf_model = RandomForestClassifier(
            n_estimators=200, max_depth=7, class_weight='balanced',
            random_state=42, n_jobs=-1,
        )
        rf_model.fit(X_tr, y_tr)
        rf_prob = rf_model.predict_proba(X_va)[:, 1]
        rf_pred = (rf_prob >= 0.5).astype(int)
        all_rows.append(eval_predictions(y_va, rf_pred, rf_prob, f'RF_fold_{held_out}'))
        rf_probs_all.append(rf_prob)
        rf_labs.append(y_va)

        # ── LSTM (optional) ──────────────────────────────────────────────────
        if train_dl:
            try:
                lstm_model, lstm_prob = train_lstm_fold(Xs_tr, y_tr, Xs_va, y_va)
                lstm_pred = (lstm_prob >= 0.5).astype(int)
                all_rows.append(eval_predictions(y_va, lstm_pred, lstm_prob,
                                                  f'LSTM_fold_{held_out}'))
                lstm_probs_all.append(lstm_prob)
                lstm_labs.append(y_va)
            except Exception as e:
                warnings.warn(f'LSTM fold {held_out} failed: {e}')

        print(f'  Fold {held_out}: XGB recall={all_rows[-1]["recall"]:.2f}  '
              f'RF recall={all_rows[-2]["recall"]:.2f}')

    return (all_rows,
            (xgb_probs_all, xgb_labs),
            (rf_probs_all, rf_labs),
            (lstm_probs_all, lstm_labs))


# ── Final model training ──────────────────────────────────────────────────────

def train_final_models(X, X_seq, y, feat_names, best_xgb_t, best_rf_t,
                       train_dl=True):
    """Train on full dataset and save models."""
    os.makedirs('models', exist_ok=True)

    scale_pos = max((y == 0).sum() / max((y == 1).sum(), 1), 1.0)

    # XGBoost
    xgb_model = xgb.XGBClassifier(
        n_estimators=300, max_depth=5, learning_rate=0.08,
        scale_pos_weight=scale_pos,
        eval_metric='logloss', verbosity=0,
        use_label_encoder=False, random_state=42,
    )
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        xgb_model.fit(X, y)
    # Attach threshold and feature names as custom attributes
    xgb_model._fall_threshold     = best_xgb_t
    xgb_model._feature_names      = feat_names
    xgb_path = 'models/fall_detector_xgb.pkl'
    joblib.dump(xgb_model, xgb_path)
    print(f'Saved XGBoost -> {xgb_path}  (threshold={best_xgb_t:.2f})')  # smoke-test fix: ASCII for cp1252

    # Random Forest
    rf_model = RandomForestClassifier(
        n_estimators=300, max_depth=7, class_weight='balanced',
        random_state=42, n_jobs=-1,
    )
    rf_model.fit(X, y)
    rf_model._fall_threshold = best_rf_t
    rf_model._feature_names  = feat_names
    rf_path = 'models/fall_detector_rf.pkl'
    joblib.dump(rf_model, rf_path)
    print(f'Saved Random Forest -> {rf_path}  (threshold={best_rf_t:.2f})')  # smoke-test fix: ASCII for cp1252

    # LSTM
    if train_dl:
        try:
            import torch
            n_features = X_seq.shape[2]
            lstm_model = _build_lstm(n_features)
            # Quick full-data training (no val set for final model)
            import torch.nn as nn
            from torch.utils.data import DataLoader, TensorDataset
            pos_weight = torch.tensor([scale_pos], dtype=torch.float32)
            criterion  = nn.BCEWithLogitsLoss(pos_weight=pos_weight)
            optimizer  = torch.optim.Adam(lstm_model.parameters(), lr=1e-3)
            X_t = torch.tensor(X_seq, dtype=torch.float32)
            y_t = torch.tensor(y, dtype=torch.float32)
            dl  = DataLoader(TensorDataset(X_t, y_t), batch_size=32, shuffle=True)
            lstm_model.train()
            for _ in range(40):
                for xb, yb in dl:
                    optimizer.zero_grad()
                    criterion(lstm_model(xb), yb).backward()
                    optimizer.step()
            lstm_model.eval()
            lstm_path = 'models/fall_detector_lstm.pt'
            torch.save(lstm_model, lstm_path)
            print(f'Saved LSTM -> {lstm_path}')  # smoke-test fix: ASCII for cp1252
        except Exception as e:
            warnings.warn(f'LSTM final training failed: {e}')

    return xgb_model, rf_model


# ── Summary printing ─────────────────────────────────────────────────────────

def print_summary(rows, model_prefix: str):
    df = pd.DataFrame(rows)
    df = df[df['model'].str.startswith(model_prefix)]
    if df.empty:
        return
    mean = df[['precision', 'recall', 'f1', 'f2', 'roc_auc', 'avg_precision']].mean()
    std  = df[['precision', 'recall', 'f1', 'f2', 'roc_auc', 'avg_precision']].std()
    print(f'\n{model_prefix}  (LOSO-CV mean ± std over {len(df)} folds):')
    for col in mean.index:
        print(f'  {col:<20s}: {mean[col]:.3f} ± {std[col]:.3f}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Train ML fall detection models with LOSO-CV.'
    )
    parser.add_argument(
        '--data', nargs='+', required=True,
        help='ML session CSV files (supports glob patterns).'
    )
    parser.add_argument('--window', type=int, default=WINDOW_SIZE,
                        help=f'Window size in frames (default {WINDOW_SIZE})')
    parser.add_argument('--stride', type=int, default=STRIDE,
                        help=f'Stride between windows (default {STRIDE})')
    parser.add_argument('--no-dl', action='store_true',
                        help='Skip LSTM training (faster, no torch required)')
    args = parser.parse_args()

    # Expand glob patterns
    csv_paths = []
    for pattern in args.data:
        expanded = glob.glob(pattern)
        csv_paths.extend(expanded if expanded else [pattern])
    csv_paths = sorted(set(csv_paths))

    if not csv_paths:
        print('[ERROR] No CSV files found.')
        return

    print(f'Found {len(csv_paths)} session CSVs:')
    for p in csv_paths:
        print(f'  {p}')

    # Feature extraction
    print('\nExtracting features...')
    X, X_seq, y, groups, feat_names, seq_names = load_and_extract(
        csv_paths, window_size=args.window, stride=args.stride
    )

    if y.sum() == 0:
        print('[ERROR] No fall windows found. Check that CSV files have '
              '"label"="fall" rows and enough frames per track.')
        return

    train_dl = not args.no_dl

    # LOSO-CV
    print(f'\nRunning LOSO-CV ({len(set(groups))} sessions)...')
    rows, xgb_cv, rf_cv, lstm_cv = run_loso_cv(
        X, X_seq, y, groups, feat_names, train_dl=train_dl
    )

    # Threshold tuning
    best_xgb_t  = threshold_from_cv(*xgb_cv)  if xgb_cv[0]  else 0.5
    best_rf_t   = threshold_from_cv(*rf_cv)    if rf_cv[0]   else 0.5
    best_lstm_t = threshold_from_cv(*lstm_cv)  if lstm_cv[0] else 0.5

    print(f'\nOptimal thresholds (F2): XGBoost={best_xgb_t:.2f}  '
          f'RF={best_rf_t:.2f}  LSTM={best_lstm_t:.2f}')

    # Print summaries
    print_summary(rows, 'XGBoost')
    print_summary(rows, 'RF')
    if train_dl:
        print_summary(rows, 'LSTM')

    # Save CV results
    os.makedirs('models', exist_ok=True)
    cv_df = pd.DataFrame(rows)
    cv_df.to_csv('models/cv_results.csv', index=False)
    print(f'\nCV results saved to models/cv_results.csv')

    # Save thresholds
    thresholds = {
        'xgb': best_xgb_t, 'rf': best_rf_t,
        'lstm': best_lstm_t,
        'window_size': args.window, 'stride': args.stride,
        'feat_names': feat_names,
    }
    with open('models/thresholds.json', 'w') as f:
        json.dump(thresholds, f, indent=2)

    # Train and save final models
    print('\nTraining final models on full dataset...')
    train_final_models(X, X_seq, y, feat_names, best_xgb_t, best_rf_t,
                       train_dl=train_dl)
    print('\nDone.')


if __name__ == '__main__':
    main()
