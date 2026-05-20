"""IR fall confirmer — passive lying-down classifier for MLX90640.

Public surface:
    IrConfirmer       — main class (calibrate, is_available, evaluate)
    IrConfirmerParams — hyperparameter dataclass (thresholds, windows)
    ConfirmerResult   — output of evaluate()
    IrRingBuffer      — internal but exported for testing
    BackgroundModel   — internal but exported for testing

Design lives in docs/superpowers/specs/2026-05-19-ir-fall-confirmer-design.md.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# ─── hyperparameters ────────────────────────────────────────────────────────


@dataclass
class IrConfirmerParams:
    # Windowing
    buffer_seconds: float = 10.0
    pre_window_s: float = 2.0
    post_window_s: float = 5.0

    # Calibration / safeguard
    calibration_seconds: float = 30.0
    safeguard_cluster_size: int = 5
    safeguard_cluster_dt: float = 5.0   # °C above frame mean
    safeguard_peak_dt: float = 8.0      # °C: max(frame) - mean(frame)
    calibration_retry_seconds: float = 60.0

    # Blob detection
    mask_sigma_k: float = 3.0           # threshold = bg_mean + k * bg_std
    min_blob_pixels: int = 10

    # Rule 0 (pre-window ghost veto)
    pre_empty_fraction: float = 0.8

    # Rule 1 (hot blob presence)
    hot_blob_min_frame_fraction: float = 0.6

    # Rule 2 (floor zone occupancy)
    floor_zone_ratio: float = 0.5       # bottom half of post-rotation image
    floor_min_frame_fraction: float = 0.7

    # Rule 3 (horizontal aspect ratio)
    horizontal_aspect_max: float = 0.7  # h/w threshold
    horizontal_min_frame_fraction: float = 0.5

    # Rule 4 (stillness, last 3s of post window)
    stillness_window_s: float = 3.0
    stillness_max_centroid_std_px: float = 4.0

    # Voting
    min_rules_passed: int = 3           # 3 of 4 rules => CONFIRMED


@dataclass
class ConfirmerResult:
    decision: str                                  # "confirmed" | "vetoed" | "failopen"
    confidence: float                              # 0..1
    features: dict[str, float] = field(default_factory=dict)
    rule_passed: dict[str, bool] = field(default_factory=dict)
    frames_used: int = 0
    pre_available: bool = False
    window_t_mono_ns: tuple[int, int] = (0, 0)
    reason: str = ""                               # human-readable cause


# ─── ring buffer ────────────────────────────────────────────────────────────


class IrRingBuffer:
    """Time-windowed buffer of IR frames keyed by monotonic ns timestamps.

    Memory is O(buffer_seconds * fps * H * W * 4 bytes). At 8 fps, 10 s,
    24x32 float32 -> ~60 KB. Fine to keep in memory.
    """

    def __init__(self, max_seconds: float):
        self._max_ns = int(max_seconds * 1e9)
        self._frames: list[np.ndarray] = []
        self._ts: list[int] = []

    def push(self, frame: np.ndarray, t_mono_ns: int) -> None:
        self._frames.append(np.asarray(frame, dtype=np.float32))
        self._ts.append(int(t_mono_ns))
        self._evict(reference_t_mono_ns=int(t_mono_ns))

    def _evict(self, reference_t_mono_ns: int) -> None:
        cutoff = reference_t_mono_ns - self._max_ns
        # Drop oldest while older than cutoff
        drop = 0
        for ts in self._ts:
            if ts < cutoff:
                drop += 1
            else:
                break
        if drop > 0:
            del self._frames[:drop]
            del self._ts[:drop]

    def count(self) -> int:
        return len(self._frames)

    def slice(self, start_t_mono_ns: int, end_t_mono_ns: int) -> tuple[np.ndarray, np.ndarray]:
        """Return (frames, t_mono_ns) inside the half-open interval
        [start_t_mono_ns, end_t_mono_ns). The start is inclusive; the end
        is exclusive so consecutive slices do not overlap on the boundary."""
        if not self._frames:
            return (np.zeros((0, 24, 32), dtype=np.float32),
                    np.zeros((0,), dtype=np.int64))
        ts_arr = np.asarray(self._ts, dtype=np.int64)
        mask = (ts_arr >= start_t_mono_ns) & (ts_arr < end_t_mono_ns)
        idxs = np.nonzero(mask)[0]
        if idxs.size == 0:
            return (np.zeros((0,) + self._frames[0].shape, dtype=np.float32),
                    np.zeros((0,), dtype=np.int64))
        frames = np.stack([self._frames[i] for i in idxs], axis=0)
        return frames, ts_arr[idxs]


# Placeholders so the unit tests at this task can import the names.
# Filled in by later tasks.


class BackgroundModel:  # noqa: D401
    """Background-model placeholder. Implemented in Task 3."""

    def __init__(self, *_, **__) -> None:
        raise NotImplementedError


class IrConfirmer:  # noqa: D401
    """IR confirmer placeholder. Implemented in Task 7."""

    def __init__(self, *_, **__) -> None:
        raise NotImplementedError
