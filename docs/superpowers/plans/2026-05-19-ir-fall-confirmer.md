# IR Fall Confirmer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a passive "lying on floor" IR confirmer that downgrades or confirms tier-1 radar fall events, plus the phase-1 kinetic exploration script, plus the dashboard/notifier wiring that supports the three new event types with fail-open behaviour.

**Architecture:** A new module `tools/ir_confirmer.py` owns a ring buffer of MLX90640 frames, an auto-calibrated background model, and a rule-based posture classifier (Rule 0 pre-veto + 4 rules with 3-of-4 voting). `radar_reader.py` queries the confirmer when its existing `FallDetector` fires tier-1, then emits `fall_confirmed | fall_failopen | fall_candidate` via `saferoom_notifier`. `dashboard_server.py` whitelists the new event types, routes only `fall_confirmed` / `fall_failopen` to Telegram, and the SPA paints them with distinct visuals (badge for fail-open). An offline log `logs/fall_events.jsonl` captures every event with its full feature payload so the tesis can compute precision/recall under three scenarios (radar-only, radar+IR strict, radar+IR fail-open).

**Tech Stack:** Python 3.13 on RPi 5, numpy, dataclasses, FastAPI (dashboard), PyQtGraph (live view, already wired with `--ir` flag), pytest for unit tests. No new runtime dependencies.

---

## File map

| Path | Action | Responsibility |
|------|--------|---------------|
| `tools/ir_confirmer.py` | **new** | `IrRingBuffer`, `BackgroundModel`, `IrConfirmer`, `IrConfirmerParams`, `ConfirmerResult` |
| `tools/explore_ir_kinetics.py` | **new** | Phase-1 A_kinetic: load labeled sessions, compute candidate kinetic features, ROC plots |
| `tools/radar_reader.py` | modify | Build the confirmer when `--ir` is active; emit new event types; add `--no-confirmer` flag; append to `logs/fall_events.jsonl` |
| `tools/dashboard_server.py` | modify | Extend `EVENT_TYPES`, `TG_CRITICAL_TYPES`, `active_alert` semantics |
| `tools/dashboard/index.html` | modify | Render `fall_confirmed` (red), `fall_failopen` (red + "IR offline" badge), `fall_candidate` (orange row, no banner) |
| `tests/__init__.py` | **new** | (empty marker) |
| `tests/test_ir_confirmer.py` | **new** | Unit tests for confirmer logic with synthetic frames |
| `docs/data_collection_protocol.md` | modify | Add the "30 s empty room at start of session" step |
| `docs/labeling_protocol.md` | modify | Clarify that the `fall` span MAY extend through post-fall lying (no vocab change) |
| `requirements.txt` | modify | Add `pytest>=7` |

---

## Task 1: Set up the test scaffold

**Files:**
- Create: `tests/__init__.py`
- Create: `tests/conftest.py`
- Modify: `requirements.txt`

- [ ] **Step 1: Create empty package marker**

Create `tests/__init__.py` (file content is just the newline):

```python
```

- [ ] **Step 2: Add a conftest that puts `tools/` on `sys.path`**

Create `tests/conftest.py`:

```python
"""pytest configuration for SafeRoom — exposes tools/ as importable."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, "..", "tools"))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
```

- [ ] **Step 3: Add pytest to requirements.txt**

Edit `requirements.txt` — append exactly one line:

```
pytest>=7
```

So the file becomes:

```
pyserial>=3.5
numpy>=1.24
matplotlib>=3.7
pytest>=7
```

- [ ] **Step 4: Install pytest locally and verify discovery**

Run from repo root:

```
pip install pytest>=7
python -m pytest tests/ -q
```

Expected: `no tests ran in 0.0Xs` (no tests yet, but discovery succeeds).

- [ ] **Step 5: Commit**

```
git add tests/__init__.py tests/conftest.py requirements.txt
git commit -m "test: add pytest scaffold with tools/ on sys.path"
```

---

## Task 2: Dataclasses + `IrRingBuffer`

**Files:**
- Create: `tools/ir_confirmer.py`
- Test: `tests/test_ir_confirmer.py`

- [ ] **Step 1: Write the failing tests for `IrRingBuffer`**

Create `tests/test_ir_confirmer.py` with this content:

```python
"""Unit tests for tools/ir_confirmer.py — synthetic frames only, no I2C."""
import numpy as np
import pytest

from ir_confirmer import (
    ConfirmerResult,
    IrConfirmerParams,
    IrRingBuffer,
)


def _frame(value: float = 22.0) -> np.ndarray:
    """24x32 uniform-temperature frame."""
    return np.full((24, 32), value, dtype=np.float32)


def test_ring_buffer_grows_then_evicts():
    buf = IrRingBuffer(max_seconds=2.0)
    # Push 3 frames spaced 0.5s apart starting at t=0
    for i in range(3):
        buf.push(_frame(20.0 + i), t_mono_ns=int(i * 0.5 * 1e9))
    assert buf.count() == 3

    # Push a frame at t=3.0s -> evicts the t=0 frame (older than 2.0s)
    buf.push(_frame(30.0), t_mono_ns=int(3.0 * 1e9))
    assert buf.count() == 3   # 0.5s, 1.0s, 3.0s still inside window from newest

    # Push another at t=4.0s -> evicts 0.5s, 1.0s
    buf.push(_frame(31.0), t_mono_ns=int(4.0 * 1e9))
    assert buf.count() == 2


def test_ring_buffer_slice_by_window():
    buf = IrRingBuffer(max_seconds=10.0)
    for i in range(20):
        buf.push(_frame(20.0 + i), t_mono_ns=int(i * 0.125 * 1e9))  # 8 fps

    # Slice from t=1.0s to t=2.0s -> 8 frames expected (1.0, 1.125, ..., 1.875)
    frames, ts = buf.slice(start_t_mono_ns=int(1.0 * 1e9),
                           end_t_mono_ns=int(2.0 * 1e9))
    assert frames.shape == (8, 24, 32)
    assert ts.shape == (8,)
    assert ts[0] == int(1.0 * 1e9)
    assert ts[-1] == int(1.875 * 1e9)


def test_confirmer_params_defaults():
    p = IrConfirmerParams()
    assert p.pre_window_s == 2.0
    assert p.post_window_s == 5.0
    assert p.calibration_seconds == 30.0
    assert p.safeguard_cluster_size == 5
    assert p.safeguard_cluster_dt == 5.0
    assert p.safeguard_peak_dt == 8.0
```

- [ ] **Step 2: Run the tests to confirm they fail**

Run:

```
python -m pytest tests/test_ir_confirmer.py -v
```

Expected: `ModuleNotFoundError: No module named 'ir_confirmer'`.

- [ ] **Step 3: Create `tools/ir_confirmer.py` with the dataclasses and ring buffer**

```python
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
        """Return (frames, t_mono_ns) inside [start, end] inclusive."""
        if not self._frames:
            return (np.zeros((0, 24, 32), dtype=np.float32),
                    np.zeros((0,), dtype=np.int64))
        ts_arr = np.asarray(self._ts, dtype=np.int64)
        mask = (ts_arr >= start_t_mono_ns) & (ts_arr <= end_t_mono_ns)
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
```

- [ ] **Step 4: Run the tests — expect pass**

```
python -m pytest tests/test_ir_confirmer.py -v
```

Expected: `3 passed`.

- [ ] **Step 5: Commit**

```
git add tools/ir_confirmer.py tests/test_ir_confirmer.py
git commit -m "ir_confirmer: dataclasses + IrRingBuffer with unit tests"
```

---

## Task 3: `BackgroundModel` with empty-room safeguard

**Files:**
- Modify: `tools/ir_confirmer.py`
- Modify: `tests/test_ir_confirmer.py`

- [ ] **Step 1: Append failing tests for `BackgroundModel`**

Append to `tests/test_ir_confirmer.py`:

```python
from ir_confirmer import BackgroundModel


def _person_frame(base_temp: float = 22.0, person_temp: float = 34.0,
                  cy: int = 12, cx: int = 16, size: int = 4) -> np.ndarray:
    f = np.full((24, 32), base_temp, dtype=np.float32)
    f[cy - size // 2: cy + size // 2 + 1,
      cx - size // 2: cx + size // 2 + 1] = person_temp
    return f


def test_background_model_learns_uniform_room():
    bg = BackgroundModel()
    for _ in range(60):  # 60 empty frames
        bg.feed(_frame(22.0) + np.random.normal(0, 0.05, (24, 32)).astype(np.float32))
    bg.finalize()
    assert bg.is_calibrated()
    assert 21.5 < float(bg.mean.mean()) < 22.5
    # std should be small (sensor noise)
    assert float(bg.std.mean()) < 0.5


def test_background_model_safeguard_rejects_person():
    bg = BackgroundModel()
    # 30 frames with a hot 5x5 person blob (max 34 over mean 22 -> dt = 12)
    for _ in range(30):
        bg.feed(_person_frame(base_temp=22.0, person_temp=34.0, size=5))
    bg.finalize()
    assert not bg.is_calibrated()
    assert "cluster" in bg.last_reject_reason.lower() or "person" in bg.last_reject_reason.lower()


def test_background_model_safeguard_tolerates_warm_ambient():
    """Hot summer day: ambient 30 °C, but no clustered hotter region."""
    bg = BackgroundModel()
    for _ in range(60):
        f = np.full((24, 32), 30.0, dtype=np.float32)
        f += np.random.normal(0, 0.1, f.shape).astype(np.float32)
        bg.feed(f)
    bg.finalize()
    assert bg.is_calibrated()


def test_background_model_safeguard_tolerates_single_noisy_pixel():
    """One isolated hot pixel (sensor glitch) should not abort calibration."""
    bg = BackgroundModel()
    for _ in range(60):
        f = np.full((24, 32), 22.0, dtype=np.float32)
        f[5, 5] = 40.0   # single very hot pixel — cluster size = 1
        bg.feed(f)
    bg.finalize()
    assert bg.is_calibrated()
```

- [ ] **Step 2: Run tests to confirm failure**

```
python -m pytest tests/test_ir_confirmer.py::test_background_model_learns_uniform_room -v
```

Expected: NotImplementedError (placeholder from Task 2).

- [ ] **Step 3: Implement `BackgroundModel`**

In `tools/ir_confirmer.py`, replace the placeholder `class BackgroundModel:` block with:

```python
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
```

- [ ] **Step 4: Run tests, expect pass**

```
python -m pytest tests/test_ir_confirmer.py -v
```

Expected: all tests in this file pass.

- [ ] **Step 5: Commit**

```
git add tools/ir_confirmer.py tests/test_ir_confirmer.py
git commit -m "ir_confirmer: BackgroundModel + empty-room safeguard"
```

---

## Task 4: Per-frame feature primitives (mask, centroid, bbox)

**Files:**
- Modify: `tools/ir_confirmer.py`
- Modify: `tests/test_ir_confirmer.py`

- [ ] **Step 1: Append failing tests**

Append to `tests/test_ir_confirmer.py`:

```python
from ir_confirmer import (
    blob_mask,
    blob_centroid,
    blob_bbox,
)


def test_blob_mask_picks_pixels_above_bg_plus_sigma():
    bg_mean = np.full((24, 32), 22.0, dtype=np.float32)
    bg_std = np.full((24, 32), 0.1, dtype=np.float32)
    frame = bg_mean.copy()
    frame[10:14, 12:18] = 30.0    # 4x6 = 24 hot pixels
    mask = blob_mask(frame, bg_mean, bg_std, sigma_k=3.0)
    assert mask.shape == (24, 32)
    assert mask.sum() == 24
    # And cool pixels are off
    assert not mask[0, 0]


def test_blob_mask_empty_when_no_signal():
    bg_mean = np.full((24, 32), 22.0, dtype=np.float32)
    bg_std = np.full((24, 32), 0.5, dtype=np.float32)
    frame = bg_mean + np.random.normal(0, 0.1, bg_mean.shape).astype(np.float32)
    mask = blob_mask(frame, bg_mean, bg_std, sigma_k=3.0)
    assert mask.sum() < 10


def test_blob_centroid_returns_geometric_center_of_mask():
    mask = np.zeros((24, 32), dtype=bool)
    mask[10:14, 12:18] = True       # rows 10..13, cols 12..17
    cy, cx = blob_centroid(mask)
    assert abs(cy - 11.5) < 1e-6
    assert abs(cx - 14.5) < 1e-6


def test_blob_centroid_nan_when_empty():
    mask = np.zeros((24, 32), dtype=bool)
    cy, cx = blob_centroid(mask)
    assert np.isnan(cy) and np.isnan(cx)


def test_blob_bbox_returns_inclusive_extents():
    mask = np.zeros((24, 32), dtype=bool)
    mask[5:9, 10:14] = True
    y0, y1, x0, x1 = blob_bbox(mask)
    assert (y0, y1, x0, x1) == (5, 8, 10, 13)
```

- [ ] **Step 2: Run, expect failure**

```
python -m pytest tests/test_ir_confirmer.py::test_blob_mask_picks_pixels_above_bg_plus_sigma -v
```

Expected: ImportError on `blob_mask` etc.

- [ ] **Step 3: Implement the helpers**

In `tools/ir_confirmer.py`, **above** the `IrConfirmer` class, add:

```python
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
```

- [ ] **Step 4: Run tests, expect pass**

```
python -m pytest tests/test_ir_confirmer.py -v
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```
git add tools/ir_confirmer.py tests/test_ir_confirmer.py
git commit -m "ir_confirmer: blob_mask / blob_centroid / blob_bbox primitives"
```

---

## Task 5: `IrConfirmer` — calibration + state queries

**Files:**
- Modify: `tools/ir_confirmer.py`
- Modify: `tests/test_ir_confirmer.py`

- [ ] **Step 1: Append failing tests**

Append to `tests/test_ir_confirmer.py`:

```python
from ir_confirmer import IrConfirmer


def test_confirmer_starts_uncalibrated():
    c = IrConfirmer(IrConfirmerParams())
    assert not c.is_calibrated()
    assert not c.is_available()


def test_confirmer_calibrates_with_clean_frames():
    p = IrConfirmerParams(calibration_seconds=2.0)
    c = IrConfirmer(p)
    for i in range(20):                          # 8 fps * 2.0s = 16 frames
        f = _frame(22.0) + np.random.normal(0, 0.05, (24, 32)).astype(np.float32)
        c.push(f, t_mono_ns=int(i * 0.125 * 1e9))
    # Allow finalize after 2s of frames
    assert c.is_calibrated()
    assert c.is_available()


def test_confirmer_rejects_calibration_with_person_and_retries():
    p = IrConfirmerParams(calibration_seconds=2.0,
                          calibration_retry_seconds=1.0)
    c = IrConfirmer(p)
    # Phase 1: person present during first calibration window
    for i in range(20):
        f = _person_frame(base_temp=22.0, person_temp=34.0, size=5)
        c.push(f, t_mono_ns=int(i * 0.125 * 1e9))
    assert not c.is_calibrated()
    assert "cluster" in c.last_calibration_reject_reason().lower()

    # Phase 2: after retry_seconds the buffer keeps growing with empty room
    # Frames between t=2.0s and t=3.0s (retry window) — clean
    for i in range(20, 30):
        f = _frame(22.0) + np.random.normal(0, 0.05, (24, 32)).astype(np.float32)
        c.push(f, t_mono_ns=int(i * 0.125 * 1e9))
    # Allow the retry trigger at t >= last + retry_seconds (1.0s)
    for i in range(30, 50):
        f = _frame(22.0) + np.random.normal(0, 0.05, (24, 32)).astype(np.float32)
        c.push(f, t_mono_ns=int(i * 0.125 * 1e9))
    assert c.is_calibrated()
```

- [ ] **Step 2: Run, expect failures**

```
python -m pytest tests/test_ir_confirmer.py -v
```

Expected: tests fail on `IrConfirmer` (placeholder NotImplementedError).

- [ ] **Step 3: Implement calibration plumbing**

In `tools/ir_confirmer.py`, replace the `IrConfirmer` placeholder block with:

```python
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
        self._last_calibration_attempt_ns: int = -1_000_000_000_000  # very negative
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
```

- [ ] **Step 4: Run tests, expect pass**

```
python -m pytest tests/test_ir_confirmer.py -v
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```
git add tools/ir_confirmer.py tests/test_ir_confirmer.py
git commit -m "ir_confirmer: IrConfirmer.push + calibration + retry on safeguard reject"
```

---

## Task 6: `IrConfirmer.evaluate()` — Rule 0 + Rules 1–4 + voting

**Files:**
- Modify: `tools/ir_confirmer.py`
- Modify: `tests/test_ir_confirmer.py`

- [ ] **Step 1: Append failing tests for `evaluate`**

Append to `tests/test_ir_confirmer.py`:

```python
def _ramp_session(c: IrConfirmer, n_empty: int, *, fps: int = 8,
                  start_idx: int = 0) -> int:
    """Push n_empty empty frames at 8 fps; return next frame index."""
    for i in range(start_idx, start_idx + n_empty):
        f = _frame(22.0) + np.random.normal(0, 0.05, (24, 32)).astype(np.float32)
        c.push(f, t_mono_ns=int(i * (1.0 / fps) * 1e9))
    return start_idx + n_empty


def _push_lying_person(c: IrConfirmer, n: int, *, fps: int = 8,
                       start_idx: int) -> int:
    """Push n frames where a horizontal blob sits in the bottom rows."""
    for i in range(start_idx, start_idx + n):
        f = np.full((24, 32), 22.0, dtype=np.float32)
        # bottom strip: rows 18..21, cols 8..24 — 4 tall x 16 wide -> horizontal
        f[18:22, 8:24] = 33.0 + np.random.normal(0, 0.1, (4, 16)).astype(np.float32)
        c.push(f, t_mono_ns=int(i * (1.0 / fps) * 1e9))
    return start_idx + n


def _push_standing_person(c: IrConfirmer, n: int, *, fps: int = 8,
                          start_idx: int) -> int:
    """Push n frames where a vertical blob sits in the top half."""
    for i in range(start_idx, start_idx + n):
        f = np.full((24, 32), 22.0, dtype=np.float32)
        # top: rows 2..14, cols 14..18 — 12 tall x 4 wide -> vertical
        f[2:14, 14:18] = 33.0 + np.random.normal(0, 0.1, (12, 4)).astype(np.float32)
        c.push(f, t_mono_ns=int(i * (1.0 / fps) * 1e9))
    return start_idx + n


def test_evaluate_confirms_when_person_is_lying_after_event():
    p = IrConfirmerParams(calibration_seconds=2.0)
    c = IrConfirmer(p)

    next_i = _ramp_session(c, 20)               # 0..19: calibration
    assert c.is_calibrated()

    next_i = _push_standing_person(c, 16, start_idx=next_i)   # 2s standing before event
    event_t_ns = next_i * int(1e9 / 8)                        # event at this index
    # Add 5s post-event of lying frames
    next_i = _push_lying_person(c, 40, start_idx=next_i)

    result = c.evaluate(event_t_ns)
    assert result.decision == "confirmed", result.reason
    assert result.confidence >= 0.75


def test_evaluate_vetoes_when_no_person_visible_post_event():
    p = IrConfirmerParams(calibration_seconds=2.0)
    c = IrConfirmer(p)

    next_i = _ramp_session(c, 20)
    # Person standing before
    next_i = _push_standing_person(c, 16, start_idx=next_i)
    event_t_ns = next_i * int(1e9 / 8)
    # Post: room becomes empty (ghost track scenario)
    next_i = _ramp_session(c, 40, start_idx=next_i)

    result = c.evaluate(event_t_ns)
    assert result.decision == "vetoed"


def test_evaluate_rule_0_vetoes_when_pre_window_is_empty():
    """Ghost track from a static reflection — no body before the event."""
    p = IrConfirmerParams(calibration_seconds=2.0)
    c = IrConfirmer(p)

    next_i = _ramp_session(c, 20)
    # Pre window: 2 s of EMPTY frames (no body visible)
    next_i = _ramp_session(c, 16, start_idx=next_i)
    event_t_ns = next_i * int(1e9 / 8)
    # Post: even if a hot blob appears later, Rule 0 already vetoes
    next_i = _push_lying_person(c, 40, start_idx=next_i)

    result = c.evaluate(event_t_ns)
    assert result.decision == "vetoed"
    assert "pre-window" in result.reason.lower() or "ghost" in result.reason.lower()


def test_evaluate_failopen_when_uncalibrated():
    c = IrConfirmer(IrConfirmerParams())
    # No frames pushed — uncalibrated
    result = c.evaluate(int(1e9))
    assert result.decision == "failopen"
```

- [ ] **Step 2: Run, expect failures**

```
python -m pytest tests/test_ir_confirmer.py -v
```

Expected: failures on the new `evaluate` tests (method does not exist).

- [ ] **Step 3: Implement `evaluate` + rules**

In `tools/ir_confirmer.py`, **inside class `IrConfirmer`**, append the following methods at the end of the class body:

```python
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
```

- [ ] **Step 4: Run tests, expect pass**

```
python -m pytest tests/test_ir_confirmer.py -v
```

Expected: all tests in the file pass.

- [ ] **Step 5: Commit**

```
git add tools/ir_confirmer.py tests/test_ir_confirmer.py
git commit -m "ir_confirmer: evaluate() with Rule 0 + 4 rules + 3-of-4 vote"
```

---

## Task 7: Wire confirmer into `radar_reader.py`

**Files:**
- Modify: `tools/radar_reader.py`

The integration: in `_read_loop`, on each frame, if the `IrConfirmer`
exists, push the corresponding IR frame via the same callback we already
use for the live panel. When a tier-1 (FAST) fall fires, evaluate and emit
the appropriate event. Also append every fall outcome to
`logs/fall_events.jsonl`.

- [ ] **Step 1: Read where the IR callback is currently wired**

Open `tools/radar_reader.py` and locate the block that constructs
`Mlx90640Capture` (around `args.ir:` in `main()` — should match Task 1
of the earlier commit `ec70a55`). Confirm the callback is `_ir_push`
which only writes to the live `ir_queue`.

- [ ] **Step 2: Add `--no-confirmer` flag and import the confirmer**

In `tools/radar_reader.py`, near the other IR CLI args, add:

```python
    parser.add_argument('--no-confirmer', action='store_true',
                        help='Disable the IR fall confirmer even if --ir is on. '
                             'Used for radar-only baseline comparisons in evaluation.')
```

At the top of the file with the other tool imports, add:

```python
try:
    from ir_confirmer import IrConfirmer, IrConfirmerParams
    _IR_CONFIRMER_AVAILABLE = True
except ImportError:
    _IR_CONFIRMER_AVAILABLE = False
```

- [ ] **Step 3: Build the `IrConfirmer` and extend the callback**

In the `if args.ir:` block where `Mlx90640Capture` is started, **replace**
the existing callback construction with this expanded version:

```python
                ir_queue = queue.Queue(maxsize=2)
                ir_rot_k = (args.ir_rotate // 90) % 4

                confirmer = None
                if (not args.no_confirmer) and _IR_CONFIRMER_AVAILABLE:
                    confirmer = IrConfirmer(IrConfirmerParams())
                    print('[IR] confirmer enabled — calibrating background...')
                elif args.no_confirmer:
                    print('[IR] confirmer DISABLED (--no-confirmer)')
                else:
                    print('[WARN] ir_confirmer not importable; confirmer off.')

                def _ir_push(frame_np, t_mono_ns):
                    # Apply the same rotation we apply for display so the
                    # confirmer reasons in image (post-rotation) coordinates.
                    rotated = np.rot90(frame_np, k=ir_rot_k) if ir_rot_k else frame_np
                    if confirmer is not None:
                        confirmer.push(rotated, t_mono_ns)
                    # Keep only the freshest frame for the live panel.
                    try:
                        while True:
                            ir_queue.get_nowait()
                    except Exception:
                        pass
                    try:
                        ir_queue.put_nowait((rotated, t_mono_ns))
                    except Exception:
                        pass
```

- [ ] **Step 4: Emit the new event types on tier-1 fall**

In `tools/radar_reader.py`, find the block in `_read_loop` that prints
`*** FALL DETECTED ***` and calls notifier (around the same area where
`fall_tids` is populated). After the existing notifier `event('fall_fast',
...)` call, add the fusion + emission logic. **Insert** the block below
right after `notifier.event('fall_fast', ...)`:

```python
                    # ── IR fusion (spec 2026-05-19) ─────────────────────
                    if confirmer is not None and not args.no_confirmer:
                        if not confirmer.is_available():
                            decision_event = 'fall_failopen'
                            cr = None
                            reason = 'ir unavailable'
                        else:
                            cr = confirmer.evaluate(frame['t_mono_ns'])
                            if cr.decision == 'confirmed':
                                decision_event = 'fall_confirmed'
                            elif cr.decision == 'failopen':
                                decision_event = 'fall_failopen'
                            else:
                                decision_event = 'fall_candidate'
                            reason = cr.reason
                        _emit_fall_event(notifier, decision_event, t,
                                         frame['t_mono_ns'], cr, reason)
                    else:
                        # No confirmer in play — keep emitting only legacy fall_fast.
                        pass
```

- [ ] **Step 5: Add `_emit_fall_event` helper near the top of `radar_reader.py`**

Add this function near the other module-level helpers in `radar_reader.py`:

```python
def _emit_fall_event(notifier, event_type: str, track: dict, t_mono_ns: int,
                     cr, reason: str) -> None:
    """Send the IR-fused fall decision to the dashboard AND append to
    logs/fall_events.jsonl for offline tesis evaluation."""
    import os
    import json

    payload = {
        'tid': int(track.get('tid', -1)),
        'x': float(track.get('x', 0.0)),
        'y': float(track.get('y', 0.0)),
        'z': float(track.get('z', 0.0)),
        'vz': float(track.get('vz', 0.0)),
        'reason': reason,
    }
    if cr is not None:
        payload['ir_decision'] = cr.decision
        payload['ir_confidence'] = round(cr.confidence, 3)
        payload['ir_features'] = {k: (None if (isinstance(v, float) and (v != v))
                                       else round(v, 4))
                                   for k, v in cr.features.items()}
        payload['ir_rules'] = cr.rule_passed
        payload['ir_pre_available'] = cr.pre_available
        payload['ir_frames_used'] = cr.frames_used

    if notifier is not None:
        notifier.event(event_type, track_id=payload['tid'], **payload)

    # Append to JSONL for tesis comparison
    os.makedirs('logs', exist_ok=True)
    row = {
        't_mono_ns': int(t_mono_ns),
        't_wall': time.time(),
        'event_type': event_type,
        'payload': payload,
    }
    with open('logs/fall_events.jsonl', 'a', encoding='utf-8') as f:
        f.write(json.dumps(row) + '\n')
```

- [ ] **Step 6: Wire `confirmer` into the read-loop signature**

Currently `_read_loop` doesn't see the local `confirmer` because it was
defined inside the `if args.plot` branch. Easiest fix: pass `confirmer`
and `args` (or just the flag) through to `_read_loop`.

Edit the `_read_loop` definition near line ~1059:

```python
def _read_loop(reader, fall_detector, logger, frame_queue, stop_event,
               sensor_h, sensor_t, use_plot, ml_detector=None, z_offset=0.0,
               confirmer=None, no_confirmer=False, notifier=None):
```

And forward the new args in both call sites in `main()`:

```python
        reader_thread = threading.Thread(
            target=_read_loop,
            args=(reader, fall_detector, logger, frame_queue, stop_event,
                  args.sensor_height, args.sensor_tilt, True),
            kwargs={
                'ml_detector': ml_detector,
                'z_offset': args.z_offset,
                'confirmer': confirmer if args.ir else None,
                'no_confirmer': args.no_confirmer,
                'notifier': getattr(args, '_notifier', None),
            },
            daemon=True,
        )
```

(`_notifier` is the existing notifier instance — if your local file
doesn't have one, leave `notifier=None` and the helper handles it.)

- [ ] **Step 7: Verify syntax with a parse check**

```
python -c "import ast; ast.parse(open('tools/radar_reader.py', encoding='utf-8').read()); print('OK')"
```

Expected: `OK`.

- [ ] **Step 8: Commit**

```
git add tools/radar_reader.py
git commit -m "radar_reader: emit fall_confirmed/failopen/candidate via IR confirmer; --no-confirmer flag; JSONL fall_events log"
```

---

## Task 8: Dashboard server accepts the three new event types

**Files:**
- Modify: `tools/dashboard_server.py`

- [ ] **Step 1: Extend the whitelist and the Telegram set**

In `tools/dashboard_server.py`, find:

```python
TG_CRITICAL_TYPES = {"fall_fast", "faint"}
EVENT_TYPES = {"presence", "presence_lost", "no_motion", "fall_fast", "faint", "heartbeat"}
```

Replace with:

```python
TG_CRITICAL_TYPES = {"fall_fast", "faint", "fall_confirmed", "fall_failopen"}
EVENT_TYPES = {
    "presence", "presence_lost", "no_motion",
    "fall_fast",            # legacy single-tier — kept for back-compat
    "fall_confirmed",       # radar + IR agreed
    "fall_failopen",        # radar fired, IR unavailable -> trust radar
    "fall_candidate",       # radar fired, IR vetoed -> log only
    "faint",
    "heartbeat",
}
```

- [ ] **Step 2: Update `active_alert` semantics**

Find this block in `post_event`:

```python
    if ev.type in {"fall_fast", "faint"}:
        state.active_alert = {
            "type": ev.type,
            "timestamp": ev.timestamp,
            "track_id": ev.track_id,
            "details": ev.details,
        }
```

Replace with:

```python
    if ev.type in {"fall_fast", "faint", "fall_confirmed", "fall_failopen"}:
        state.active_alert = {
            "type": ev.type,
            "timestamp": ev.timestamp,
            "track_id": ev.track_id,
            "details": ev.details,
        }
    # fall_candidate intentionally does NOT set active_alert (log only).
```

- [ ] **Step 3: Smoke check that the server still imports**

```
python -c "import ast; ast.parse(open('tools/dashboard_server.py', encoding='utf-8').read()); print('OK')"
```

Expected: `OK`.

- [ ] **Step 4: Commit**

```
git add tools/dashboard_server.py
git commit -m "dashboard: accept fall_confirmed / fall_failopen / fall_candidate"
```

---

## Task 9: Dashboard UI renders the three event types distinctly

**Files:**
- Modify: `tools/dashboard/index.html`

- [ ] **Step 1: Locate where events are rendered**

In `tools/dashboard/index.html`, find the JS code that renders an event
row from a server-sent `event` message (search for `"fall_fast"` or for
`renderEvent` / `appendEvent`). Note the structure: a function that
receives an event object and chooses a class/colour.

- [ ] **Step 2: Add per-type styling and the FAILOPEN badge**

Locate the CSS block. Append:

```css
.event-fall-confirmed { background: #b00020; color: white; font-weight: bold; }
.event-fall-failopen  { background: #b00020; color: white; font-weight: bold; }
.event-fall-failopen::after {
  content: "IR offline";
  background: #ffd54a;
  color: #2c2c2c;
  font-size: 0.75em;
  padding: 1px 6px;
  margin-left: 8px;
  border-radius: 4px;
  vertical-align: middle;
}
.event-fall-candidate { background: #f3a847; color: #1a1a1a; font-style: italic; }
```

Locate the function (call it `renderEvent`) that maps an event type to a
CSS class. Add the new mappings inside its branch logic:

```javascript
function eventCssClass(ev) {
  switch (ev.type) {
    case 'fall_confirmed': return 'event-fall-confirmed';
    case 'fall_failopen':  return 'event-fall-failopen';
    case 'fall_candidate': return 'event-fall-candidate';
    case 'fall_fast':      return 'event-fall-confirmed';   // legacy alias
    case 'faint':          return 'event-faint';
    default:               return 'event-default';
  }
}
```

If a function with this responsibility already exists, **extend it**
rather than create a new one — keep the existing naming.

- [ ] **Step 3: Update the active-banner logic to label the alert type**

Find where `state.active_alert` is rendered. Where it currently shows
`active_alert.type.toUpperCase()` (or equivalent), prepend a human label:

```javascript
const ALERT_LABELS = {
  fall_confirmed: 'FALL (confirmed by IR)',
  fall_failopen:  'FALL (IR offline)',
  fall_fast:      'FALL',
  faint:          'FAINT',
};
const label = ALERT_LABELS[active_alert.type] || active_alert.type.toUpperCase();
```

(Adjust the variable name to match the existing template; the **rule**
is: don't add a second alert section, just label the existing one.)

- [ ] **Step 4: Smoke test the page renders**

Run the dashboard locally and POST a synthetic event:

```
~/SafeRoom/.venv/bin/python ~/SafeRoom/tools/dashboard_server.py &
sleep 2
curl -s -X POST http://localhost:8000/event -H 'Content-Type: application/json' \
  -d '{"type":"fall_failopen","timestamp":'"$(date +%s.%N)"',"details":{"reason":"ir unavailable"}}'
```

Open `http://localhost:8000/` in a browser and confirm the row shows up
with the yellow "IR offline" badge. Stop the dashboard:

```
kill %1
```

- [ ] **Step 5: Commit**

```
git add tools/dashboard/index.html
git commit -m "dashboard: visually distinguish fall_confirmed / failopen (badge) / candidate"
```

---

## Task 10: `explore_ir_kinetics.py` — phase 1 of A_kinetic

**Files:**
- Create: `tools/explore_ir_kinetics.py`

- [ ] **Step 1: Write the script**

Create `tools/explore_ir_kinetics.py`:

```python
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
```

- [ ] **Step 2: Syntax check**

```
python -c "import ast; ast.parse(open('tools/explore_ir_kinetics.py', encoding='utf-8').read()); print('OK')"
```

Expected: `OK`.

- [ ] **Step 3: Dry-run on the synthetic session created earlier**

```
python tools/explore_ir_kinetics.py sessions/ --out figures/ir_kinetics_synth
```

Expected: prints feature AUCs (any numbers, since synthetic data has only
one labeled fall), creates `figures/ir_kinetics_synth/features.csv` and
`roc.png`, exits 0.

- [ ] **Step 4: Commit**

```
git add tools/explore_ir_kinetics.py
git commit -m "explore_ir_kinetics: phase-1 A_kinetic feature extraction + ROC plots"
```

---

## Task 11: Documentation updates

**Files:**
- Modify: `docs/data_collection_protocol.md`
- Modify: `docs/labeling_protocol.md`

- [ ] **Step 1: Add the empty-room calibration step to data_collection_protocol.md**

In `docs/data_collection_protocol.md`, locate the **"Procedimiento por
sesión"** section and modify the bullet "2. Iniciar:" to mention the
calibration phase:

Replace step 2 with:

```markdown
2. **Iniciar** (la cámara IR auto-calibra los primeros 30 s — la habitación
   debe estar **vacía** durante ese intervalo, después puede entrar el sujeto):
   ```
   python tools/session_recorder.py \
     --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
     --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
     --duration 30 --ir-hz 16 \
     --name <sujeto>_<actividad>_<posicion> \
     --subject <sujeto> \
     --notes "<orientacion>,<ropa>,<hora>"
   ```
   Espera al mensaje `[IR] calibration OK` antes de entrar a la habitación.
   Si ves `[IR] calibration aborted — hot pixels detected`, sal de la
   habitación y espera 60 s — el script reintenta automáticamente.
```

- [ ] **Step 2: Clarify fall span in labeling_protocol.md**

In `docs/labeling_protocol.md`, find the row for `fall` in the
**"Vocabulario"** table. The text already says "Caída involuntaria con
impacto / colapso al suelo". No change to the table.

But under **"Criterios de frontera"**, the line that begins with
"**Final de `fall`**:" already says to include the post-fall lying period.
**No change needed** — the IR confirmer evaluates a 5 s post-window that
matches this convention exactly.

Append a new bullet to that section to make the IR alignment explicit:

```markdown
- **Coherencia con el IR confirmer**: el confirmer evalúa los 5 s
  posteriores al evento del radar. Si la etiqueta `fall` cubre ese
  intervalo completo (cuerpo en el suelo), el IR confirmará. Si el
  etiquetador termina la etiqueta antes (p.ej. en el instante del
  impacto), la métrica de fusión penalizará incorrectamente al IR.
  **Regla**: extender `fall` durante al menos 5 s tras el impacto.
```

- [ ] **Step 3: Commit**

```
git add docs/data_collection_protocol.md docs/labeling_protocol.md
git commit -m "docs: 30s IR calibration step + fall span >= 5s post-impact for fusion coherence"
```

---

## Task 12: End-to-end smoke validation on the Pi

**Files:** (no file changes; this task verifies everything together.)

- [ ] **Step 1: Push the branch and pull on the Pi**

```
git push
# In the Pi shell:
ssh guillermo@100.110.129.108 'cd ~/SafeRoom && git pull --ff-only && ~/SafeRoom/.venv/bin/pip install -q pytest>=7'
```

- [ ] **Step 2: Run unit tests on the Pi**

```
ssh guillermo@100.110.129.108 'cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python -m pytest tests/ -v'
```

Expected: all tests pass.

- [ ] **Step 3: Run a 60 s live session with confirmer**

(Run from the Pi screen-share session, not over SSH, because of the Qt window):

```
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/radar_reader.py \
  --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
  --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
  --plot --ir --ir-hz 16 --ir-rotate 90
```

Expected console flow:
1. `[IR] confirmer enabled — calibrating background...`
2. After ~30 s: implicit calibration OK (no further messages mean ready).
3. Wave a hand briefly → may trigger a tier-1 fall → log line indicating
   `fall_confirmed` or `fall_candidate` based on whether the hand looked
   like a person on the floor.

- [ ] **Step 4: Verify the JSONL log was written**

After closing the window:

```
ssh guillermo@100.110.129.108 'tail -3 ~/SafeRoom/logs/fall_events.jsonl'
```

Expected: one JSON object per fall event, each containing
`event_type`, `payload.ir_decision`, `payload.ir_features`.

- [ ] **Step 5: Verify the dashboard sees the new event types**

Open `http://100.110.129.108:8000/` in a browser (or the Pi browser) after
starting the dashboard:

```
ssh guillermo@100.110.129.108 '~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py &'
```

Trigger a synthetic event:

```
curl -X POST http://100.110.129.108:8000/event -H 'Content-Type: application/json' \
  -d '{"type":"fall_failopen","timestamp":'$(date +%s.%N)',"track_id":1,"details":{"reason":"smoke test"}}'
curl -X POST http://100.110.129.108:8000/event -H 'Content-Type: application/json' \
  -d '{"type":"fall_candidate","timestamp":'$(date +%s.%N)',"track_id":1,"details":{"reason":"smoke test"}}'
curl -X POST http://100.110.129.108:8000/event -H 'Content-Type: application/json' \
  -d '{"type":"fall_confirmed","timestamp":'$(date +%s.%N)',"track_id":1,"details":{"reason":"smoke test"}}'
```

Expected: dashboard shows one red row (confirmed), one red row with yellow
"IR offline" badge (failopen), and one orange italic row (candidate). The
active alert banner reads `FALL (confirmed by IR)` when the confirmed
event is the latest.

- [ ] **Step 6: Final tag / no commit needed**

This task introduces no code; it is a verification gate. If anything
fails, open a follow-up task rather than fix in this plan.

---

## Self-review

**Spec coverage**: every section of the spec maps to a task:

- Goals: covered by Task 7 (fusion), Task 10 (kinetic exploration), and the JSONL log in Task 7.
- Non-goals: respected — no IR-only detector in v1 (Task 7), no kinetic detector (only exploration).
- Architecture diagram: realised in Task 7 (radar_reader owns confirmer).
- Components `IrConfirmer`, `IrConfirmerParams`, `ConfirmerResult`, `IrRingBuffer`: Tasks 2–6.
- Rule 0 + Rules 1-4 + 3-of-4 voting: Task 6.
- Fusion policy (3 event types + fail-open): Tasks 7, 8.
- Calibration UX (30 s + safeguard + retry): Tasks 3, 5; documented in Task 11.
- JSONL logging for tesis evaluation: Task 7 step 5.
- A_kinetic phased: Task 10.
- LogReg upgrade for B: documented in spec; no implementation task (correctly post-dataset).
- Dashboard / index.html: Tasks 8, 9.
- Docs updates: Task 11.
- `--no-confirmer` flag: Task 7 step 2.
- Resolved questions (badge, silent retry, --no-confirmer): all in code via Tasks 7, 8, 9.

**Placeholder scan**: every code block contains the full code; every test
contains assertions; every shell command is exact. No `TBD` / `TODO` /
`implement later`.

**Type consistency**: `ConfirmerResult.decision` is always one of
`"confirmed" | "vetoed" | "failopen"` across Tasks 2, 6, 7. `IrConfirmer`
public methods `push`, `is_calibrated`, `is_available`, `evaluate`,
`last_calibration_reject_reason` match between Tasks 5–7. `IrConfirmerParams`
field names (`safeguard_cluster_size`, `safeguard_cluster_dt`,
`safeguard_peak_dt`, `floor_zone_ratio`, etc.) are used identically in
Tasks 3, 5, 6, and the spec.
