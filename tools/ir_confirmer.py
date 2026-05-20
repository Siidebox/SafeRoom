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


class BackgroundModel:
    """Per-pixel mean+std from N frames of (assumed empty) room.

    Validates emptiness at finalize() using a clustered-hot-region rule
    that tolerates uniformly warm ambient and single noisy pixels.
    """

    def __init__(self, params: Optional[IrConfirmerParams] = None) -> None:
        self._params = params or IrConfirmerParams()
        self._buf: list[np.ndarray] = []
        self.mean: np.ndarray | None = None
        self.std: np.ndarray | None = None
        self._calibrated = False
        self.last_reject_reason: str = ""

    def feed(self, frame: np.ndarray) -> None:
        self._buf.append(np.asarray(frame, dtype=np.float32))

    def finalize(self) -> bool:
        """Compute mean/std, run safeguard. Return True if calibration accepted."""
        if not self._buf:
            self._calibrated = False
            self.last_reject_reason = "no frames"
            return False

        stack = np.stack(self._buf, axis=0)               # (N, H, W)
        self.mean = stack.mean(axis=0).astype(np.float32)
        # ddof=0 to avoid NaN with N=1
        self.std = stack.std(axis=0, ddof=0).astype(np.float32)

        # Safeguard: on the latest frame check for a clustered hot region.
        # Using the last frame is enough — if the person is moving the cluster
        # appears somewhere; if static, all frames have it.
        last = stack[-1]
        frame_mean = float(last.mean())
        peak_dt = float(last.max() - frame_mean)
        cluster_count = int(np.sum(last > frame_mean + self._params.safeguard_cluster_dt))

        if (cluster_count >= self._params.safeguard_cluster_size
                and peak_dt >= self._params.safeguard_peak_dt):
            self._calibrated = False
            self.last_reject_reason = (
                f"clustered hot region detected: {cluster_count} pixels "
                f"> mean+{self._params.safeguard_cluster_dt:.1f}°C, "
                f"peak Δ={peak_dt:.1f}°C"
            )
            return False

        self._calibrated = True
        self.last_reject_reason = ""
        # Free per-frame buffer; keep aggregates.
        self._buf = []
        return True

    def is_calibrated(self) -> bool:
        return self._calibrated


# ─── per-frame feature primitives ───────────────────────────────────────────


def blob_mask(frame: np.ndarray, bg_mean: np.ndarray, bg_std: np.ndarray,
              sigma_k: float = 3.0) -> np.ndarray:
    """Boolean mask of pixels significantly hotter than background."""
    threshold = bg_mean + sigma_k * bg_std
    return frame > threshold


def blob_centroid(mask: np.ndarray) -> tuple[float, float]:
    """Unweighted centroid (cy, cx) of True pixels. NaNs when mask is empty."""
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return (float("nan"), float("nan"))
    return (float(ys.mean()), float(xs.mean()))


def blob_bbox(mask: np.ndarray) -> tuple[int, int, int, int]:
    """Inclusive (y_min, y_max, x_min, x_max). Returns (-1,-1,-1,-1) if empty."""
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return (-1, -1, -1, -1)
    return (int(ys.min()), int(ys.max()), int(xs.min()), int(xs.max()))


class IrConfirmer:
    """Reactive IR posture confirmer. Owns ring buffer + background model.

    Lifecycle:
        c = IrConfirmer(params)
        # On every IR frame received from Mlx90640Capture:
        c.push(frame_np, t_mono_ns)
        # When radar fires a tier-1 fall event:
        if not c.is_available():
            decision = "failopen"
        else:
            result = c.evaluate(radar_t_mono_ns)
    """

    def __init__(self, params: Optional[IrConfirmerParams] = None) -> None:
        self._params = params or IrConfirmerParams()
        self._buf = IrRingBuffer(max_seconds=self._params.buffer_seconds)
        self._bg = BackgroundModel(self._params)
        self._calibration_start_ns: int | None = None
        self._last_push_ns: int = 0
        self._last_reject: str = ""

    # ── public surface ──

    def push(self, frame: np.ndarray, t_mono_ns: int) -> None:
        self._buf.push(frame, t_mono_ns)
        self._last_push_ns = int(t_mono_ns)
        self._maybe_calibrate()

    def is_calibrated(self) -> bool:
        return self._bg.is_calibrated()

    def is_available(self) -> bool:
        """Fail-open feed. False when calibration not yet OK."""
        return self._bg.is_calibrated()

    def last_calibration_reject_reason(self) -> str:
        return self._last_reject

    # ── calibration internals ──

    def _maybe_calibrate(self) -> None:
        if self._bg.is_calibrated():
            return
        if self._calibration_start_ns is None:
            self._calibration_start_ns = self._last_push_ns
            return

        elapsed_s = (self._last_push_ns - self._calibration_start_ns) / 1e9
        if elapsed_s < self._params.calibration_seconds:
            return

        # Try to finalize. Use the latest `calibration_seconds` of frames.
        start_ns = self._last_push_ns - int(self._params.calibration_seconds * 1e9)
        frames, _ = self._buf.slice(start_ns, self._last_push_ns)
        if frames.shape[0] == 0:
            return

        self._bg = BackgroundModel(self._params)
        for f in frames:
            self._bg.feed(f)
        ok = self._bg.finalize()
        if not ok:
            self._last_reject = self._bg.last_reject_reason
            # schedule retry: reset calibration_start so next window opens after retry_seconds
            self._calibration_start_ns = (
                self._last_push_ns
                + int(self._params.calibration_retry_seconds * 1e9)
                - int(self._params.calibration_seconds * 1e9)
            )

    # ── evaluate ──

    def evaluate(self, radar_t_mono_ns: int) -> ConfirmerResult:
        if not self._bg.is_calibrated():
            return ConfirmerResult(
                decision="failopen",
                confidence=0.0,
                reason="background not calibrated",
                window_t_mono_ns=(radar_t_mono_ns, radar_t_mono_ns),
            )

        p = self._params
        pre_start  = radar_t_mono_ns - int(p.pre_window_s  * 1e9)
        pre_end    = radar_t_mono_ns
        post_start = radar_t_mono_ns
        post_end   = radar_t_mono_ns + int(p.post_window_s * 1e9)

        pre_frames, _ = self._buf.slice(pre_start, pre_end)
        post_frames, post_ts = self._buf.slice(post_start, post_end)

        pre_available = pre_frames.shape[0] >= 4   # need ~0.5s minimum

        # Rule 0 — pre-window ghost veto
        if pre_available:
            pre_masks = np.stack([
                blob_mask(f, self._bg.mean, self._bg.std, p.mask_sigma_k)
                for f in pre_frames
            ], axis=0)
            pre_pixel_counts = pre_masks.reshape(pre_masks.shape[0], -1).sum(axis=1)
            empty_frac = float(np.mean(pre_pixel_counts < p.min_blob_pixels))
            if empty_frac >= p.pre_empty_fraction:
                return ConfirmerResult(
                    decision="vetoed",
                    confidence=0.0,
                    reason=("rule-0 ghost-track veto: pre-window empty in "
                            f"{empty_frac:.0%} of frames"),
                    features={"pre_empty_fraction": empty_frac},
                    pre_available=True,
                    frames_used=int(pre_frames.shape[0] + post_frames.shape[0]),
                    window_t_mono_ns=(pre_start, post_end),
                )

        if post_frames.shape[0] == 0:
            return ConfirmerResult(
                decision="vetoed",
                confidence=0.0,
                reason="no post-event frames available",
                pre_available=pre_available,
                window_t_mono_ns=(pre_start, post_end),
            )

        # Per-frame post-window quantities
        post_masks = np.stack([
            blob_mask(f, self._bg.mean, self._bg.std, p.mask_sigma_k)
            for f in post_frames
        ], axis=0)
        post_counts = post_masks.reshape(post_masks.shape[0], -1).sum(axis=1)

        # Rule 1 — hot blob presence
        hot_frac = float(np.mean(post_counts >= p.min_blob_pixels))
        rule1 = hot_frac >= p.hot_blob_min_frame_fraction

        # Centroids / bboxes (only for frames with valid blobs)
        H, W = post_masks.shape[1], post_masks.shape[2]
        floor_zone_y_min = int(H * (1.0 - p.floor_zone_ratio))   # bottom half

        floor_hits = 0
        horiz_hits = 0
        floor_valid = 0
        horiz_valid = 0
        cy_list: list[float] = []
        cx_list: list[float] = []
        ts_for_centroids: list[int] = []
        for k in range(post_masks.shape[0]):
            mask_k = post_masks[k]
            if mask_k.sum() < p.min_blob_pixels:
                continue
            cy, cx = blob_centroid(mask_k)
            y0, y1, x0, x1 = blob_bbox(mask_k)
            cy_list.append(cy)
            cx_list.append(cx)
            ts_for_centroids.append(int(post_ts[k]))
            floor_valid += 1
            if cy >= floor_zone_y_min:
                floor_hits += 1
            horiz_valid += 1
            bbox_h = max(1, y1 - y0 + 1)
            bbox_w = max(1, x1 - x0 + 1)
            aspect = bbox_h / bbox_w
            if aspect < p.horizontal_aspect_max:
                horiz_hits += 1

        # Rule 2 — floor zone occupancy (fraction of valid-blob frames in floor zone)
        floor_frac = (floor_hits / floor_valid) if floor_valid else 0.0
        rule2 = floor_frac >= p.floor_min_frame_fraction

        # Rule 3 — horizontal aspect
        horiz_frac = (horiz_hits / horiz_valid) if horiz_valid else 0.0
        rule3 = horiz_frac >= p.horizontal_min_frame_fraction

        # Rule 4 — stillness in last stillness_window_s of post
        stillness_cutoff = radar_t_mono_ns + int(
            (p.post_window_s - p.stillness_window_s) * 1e9)
        last_cy = [cy for cy, ts in zip(cy_list, ts_for_centroids)
                   if ts >= stillness_cutoff and not np.isnan(cy)]
        last_cx = [cx for cx, ts in zip(cx_list, ts_for_centroids)
                   if ts >= stillness_cutoff and not np.isnan(cx)]
        if len(last_cy) >= 3:
            centroid_std = float(
                np.sqrt(np.var(last_cy) + np.var(last_cx))
            )
        else:
            centroid_std = float("nan")
        rule4 = (not np.isnan(centroid_std)
                 and centroid_std <= p.stillness_max_centroid_std_px)

        rules = {
            "rule1_hot_blob": rule1,
            "rule2_floor_zone": rule2,
            "rule3_horizontal": rule3,
            "rule4_stillness": rule4,
        }
        passed = sum(1 for v in rules.values() if v)
        confidence = passed / 4.0
        decision = "confirmed" if passed >= p.min_rules_passed else "vetoed"
        reason = ("confirmed by 3-of-4 vote" if decision == "confirmed"
                  else f"only {passed}/4 rules passed")

        return ConfirmerResult(
            decision=decision,
            confidence=confidence,
            features={
                "hot_blob_fraction": hot_frac,
                "floor_centroid_fraction": floor_frac,
                "horizontal_fraction": horiz_frac,
                "centroid_std_px": centroid_std,
            },
            rule_passed=rules,
            frames_used=int(pre_frames.shape[0] + post_frames.shape[0]),
            pre_available=pre_available,
            window_t_mono_ns=(pre_start, post_end),
            reason=reason,
        )
