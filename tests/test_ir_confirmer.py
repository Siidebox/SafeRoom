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
