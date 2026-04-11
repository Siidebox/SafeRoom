#!/usr/bin/env python3
"""
ml_inference.py — Real-time ML fall detection for radar_reader integration.

MlFallDetector maintains per-track ring buffers and runs the trained model
every `stride` frames once the buffer has `window_size` frames.

Supports:
    - XGBoost model (.pkl, saved with joblib)
    - PyTorch LSTM/CNN model (.pt)

Usage:
    from ml_inference import MlFallDetector

    detector = MlFallDetector('models/fall_detector_xgb.pkl')

    # In read loop, once per frame per track:
    is_fall, prob = detector.update(tid, track_dict, height_dict, pc_stats_dict)
    if is_fall:
        print(f'ML: fall detected on track {tid} (p={prob:.2f})')
"""

import collections
import os
from pathlib import Path

import numpy as np

# Feature extraction — reuse the same logic used during training
from feature_engineering import extract_features_from_window, SEQ_FEATURE_COLS
import pandas as pd


# ── Model loader ─────────────────────────────────────────────────────────────

def _load_model(path: str):
    """
    Load a model from path.
    - .pkl → joblib (XGBoost or sklearn)
    - .pt  → PyTorch
    Returns (model, model_type) where model_type is 'sklearn' or 'torch'.
    """
    ext = Path(path).suffix.lower()

    if ext == '.pkl':
        try:
            import joblib
        except ImportError:
            raise ImportError('joblib is required for .pkl models: pip install joblib')
        model = joblib.load(path)
        return model, 'sklearn'

    elif ext == '.pt':
        try:
            import torch
        except ImportError:
            raise ImportError('torch is required for .pt models: pip install torch')
        model = torch.load(path, map_location='cpu')
        model.eval()
        return model, 'torch'

    else:
        raise ValueError(f'Unknown model extension: {ext}. Expected .pkl or .pt')


# ── Per-track frame buffer ───────────────────────────────────────────────────

class _TrackBuffer:
    """Ring buffer that stores the last window_size frames for one track."""

    def __init__(self, window_size: int):
        self._size  = window_size
        self._buf   = collections.deque(maxlen=window_size)
        self._since_last_pred = 0   # frames since last prediction

    def push(self, row: dict):
        self._buf.append(row)
        self._since_last_pred += 1

    @property
    def ready(self) -> bool:
        return len(self._buf) == self._size

    def get_window_df(self) -> pd.DataFrame:
        return pd.DataFrame(list(self._buf))

    def reset_stride(self):
        self._since_last_pred = 0

    def should_predict(self, stride: int) -> bool:
        return self.ready and self._since_last_pred >= stride


# ── Main class ───────────────────────────────────────────────────────────────

class MlFallDetector:
    """
    Real-time ML fall detector. Mirrors FallDetector's interface:
      is_fall, prob = detector.update(tid, track, height, pc_stats)

    Parameters
    ----------
    model_path : str
        Path to a trained model file (.pkl for XGBoost/RF, .pt for PyTorch).
    window_size : int
        Sliding window length in frames (must match training, default 30).
    stride : int
        Frames between predictions per track (default 5).
    threshold : float
        Fall probability threshold for binary decision (default 0.5).
        Lower → more sensitive (fewer missed falls, more false alarms).
    cooldown : float
        Minimum seconds between consecutive fall detections per track.
    """

    def __init__(self, model_path: str, window_size: int = 30,
                 stride: int = 5, threshold: float = 0.50,
                 cooldown: float = 5.0):
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'Model not found: {model_path}')

        self._model, self._model_type = _load_model(model_path)
        self._window_size = window_size
        self._stride      = stride
        self._threshold   = threshold
        self._cooldown    = cooldown

        self._buffers     = {}   # tid → _TrackBuffer
        self._last_det    = {}   # tid → timestamp of last fall detection
        self._last_prob   = {}   # tid → last predicted probability

        import time
        self._time = time
        print(f'[MlFallDetector] Loaded {model_path} ({self._model_type})  '
              f'window={window_size} stride={stride} threshold={threshold:.2f}')

    def _init_tid(self, tid: int):
        if tid not in self._buffers:
            self._buffers[tid]  = _TrackBuffer(self._window_size)
            self._last_det[tid] = 0.0
            self._last_prob[tid] = 0.0

    def update(self, tid: int, track: dict, height: dict,
               pc_stats: dict) -> tuple:
        """
        Call once per frame per track.

        Parameters
        ----------
        tid      : track ID (int)
        track    : dict with keys x, y, z, vx, vy, vz, ax, ay, az, g, confidence
        height   : dict with keys maxZ, minZ   (from TLV 1012)
        pc_stats : dict from compute_pc_stats() in ml_logger

        Returns
        -------
        (is_fall: bool, probability: float)
        """
        self._init_tid(tid)
        buf = self._buffers[tid]

        max_z    = height.get('maxZ', 0.0)
        min_z    = height.get('minZ', 0.0)
        height_m = max_z - min_z

        row = {
            'x': track.get('x', 0.0), 'y': track.get('y', 0.0),
            'z': track.get('z', 0.0),
            'vx': track.get('vx', 0.0), 'vy': track.get('vy', 0.0),
            'vz': track.get('vz', 0.0),
            'ax': track.get('ax', 0.0), 'ay': track.get('ay', 0.0),
            'az': track.get('az', 0.0),
            'g': track.get('g', 1.0), 'confidence': track.get('confidence', 1.0),
            'maxZ': max_z, 'minZ': min_z, 'height_m': height_m,
            **pc_stats,
        }
        buf.push(row)

        if not buf.should_predict(self._stride):
            return False, self._last_prob[tid]

        buf.reset_stride()
        win_df = buf.get_window_df()
        prob   = self._predict_proba(win_df)
        self._last_prob[tid] = prob

        now = self._time.time()
        in_cooldown = (now - self._last_det[tid]) < self._cooldown
        is_fall = (not in_cooldown) and (prob >= self._threshold)

        if is_fall:
            self._last_det[tid] = now

        return is_fall, prob

    def _predict_proba(self, win_df: pd.DataFrame) -> float:
        """Run the model on one window and return fall probability."""
        if self._model_type == 'sklearn':
            feat_dict = extract_features_from_window(win_df)
            # Build feature vector in training column order
            if hasattr(self._model, 'feature_names_in_'):
                names = list(self._model.feature_names_in_)
            elif hasattr(self._model, '_feature_names'):
                names = list(self._model._feature_names)
            else:
                names = list(feat_dict.keys())
            x = np.array([[feat_dict.get(n, 0.0) for n in names]],
                          dtype=np.float32)
            try:
                prob = float(self._model.predict_proba(x)[0, 1])
            except AttributeError:
                # Some sklearn estimators only have predict()
                prob = float(self._model.predict(x)[0])
            return prob

        elif self._model_type == 'torch':
            import torch
            seq_cols = [c for c in SEQ_FEATURE_COLS if c in win_df.columns]
            arr = win_df[seq_cols].values.astype(np.float32)
            arr = np.where(np.isfinite(arr), arr, 0.0)
            # Pad if needed
            if arr.shape[1] < len(SEQ_FEATURE_COLS):
                pad = np.zeros((arr.shape[0], len(SEQ_FEATURE_COLS) - arr.shape[1]),
                               dtype=np.float32)
                arr = np.hstack([arr, pad])
            x = torch.tensor(arr).unsqueeze(0)  # (1, window_size, n_features)
            with torch.no_grad():
                out = self._model(x)
                prob = float(torch.sigmoid(out).squeeze())
            return prob

        return 0.0

    def cleanup_old_tracks(self, active_tids: set):
        """Remove state for tracks that are no longer active."""
        for tid in list(self._buffers.keys()):
            if tid not in active_tids:
                del self._buffers[tid]
                del self._last_det[tid]
                del self._last_prob[tid]

    def last_prob(self, tid: int) -> float:
        return self._last_prob.get(tid, 0.0)
