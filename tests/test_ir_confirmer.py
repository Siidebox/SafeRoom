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
    buf = IrRingBuffer(max_seconds=2.5)
    # Push 3 frames spaced 0.5s apart starting at t=0
    for i in range(3):
        buf.push(_frame(20.0 + i), t_mono_ns=int(i * 0.5 * 1e9))
    assert buf.count() == 3

    # Push a frame at t=3.0s. Cutoff = 3.0 - 2.5 = 0.5s.
    # The t=0 frame (ts < cutoff) is evicted; 0.5, 1.0, 3.0 remain.
    buf.push(_frame(30.0), t_mono_ns=int(3.0 * 1e9))
    assert buf.count() == 3

    # Push a frame at t=4.0s. Cutoff = 1.5s -> evicts 0.5 and 1.0.
    # Remaining: 3.0, 4.0.
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
