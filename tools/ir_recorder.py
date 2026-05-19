"""
ir_recorder.py — Record MLX90640 thermal frames to a .npz file.

Two ways to use:

1. CLI (standalone, sin radar):
       python tools/ir_recorder.py --out logs/ir_test.npz --duration 60

2. Programmatic (used by session_recorder.py for synchronized radar+IR):
       from ir_recorder import Mlx90640Capture
       cap = Mlx90640Capture(refresh_hz=16)
       cap.start()                    # spawns background thread
       ...
       frames, t_mono_ns, t_wall, bad = cap.stop_and_collect()

Output .npz schema (CLI mode):
    frames          : float32 (N, 24, 32) — temperatures in °C
    timestamps      : float64 (N,)        — wall-clock Unix time at end of frame
    t_mono_ns       : int64   (N,)        — monotonic ns at end of frame
    refresh_rate_hz : scalar
    bad_frames      : int — frames lost to I2C read errors
"""

import argparse
import os
import sys
import threading
import time

import numpy as np


# Adafruit deps are RPi-only. Import lazily so this module can be imported on
# a PC for testing/inspection without the I2C hardware present.
try:
    import board
    import busio
    import adafruit_mlx90640
    _MLX_AVAILABLE = True
except Exception as _e:  # noqa: BLE001
    _MLX_AVAILABLE = False
    _MLX_IMPORT_ERROR = _e


REFRESH_RATES_LAZY = {  # key → attribute name (resolved on first use)
    1:  'REFRESH_1_HZ',
    2:  'REFRESH_2_HZ',
    4:  'REFRESH_4_HZ',
    8:  'REFRESH_8_HZ',
    16: 'REFRESH_16_HZ',
    32: 'REFRESH_32_HZ',
}


def _resolve_refresh(hz: int):
    if not _MLX_AVAILABLE:
        raise RuntimeError(
            f"MLX90640 driver not available: {_MLX_IMPORT_ERROR}. "
            "Install with: pip install adafruit-circuitpython-mlx90640 lgpio"
        )
    if hz not in REFRESH_RATES_LAZY:
        raise ValueError(f"Unsupported refresh rate: {hz}. "
                         f"Choose one of {sorted(REFRESH_RATES_LAZY)}")
    return getattr(adafruit_mlx90640.RefreshRate, REFRESH_RATES_LAZY[hz])


# ── Reusable capture class ───────────────────────────────────────────────────

class Mlx90640Capture:
    """
    Threaded MLX90640 capture. Single shared monotonic clock with the caller
    (use time.monotonic_ns() in both processes for radar+IR alignment).

    Lifecycle:
        cap = Mlx90640Capture(refresh_hz=16)
        cap.start()
        ...                       # do other work (record radar etc.)
        cap.stop_and_collect()    # returns (frames, t_mono_ns, t_wall, bad)

    The thread loops mlx.getFrame() as fast as the sensor will produce frames.
    Effective output rate is ~half of refresh_hz (sensor returns alternating
    sub-pages and the driver merges them every two reads).
    """

    def __init__(self, refresh_hz: int = 16):
        self.refresh_hz = refresh_hz
        self._mlx       = None
        self._buf       = [0.0] * 768
        self._frames    = []
        self._t_mono    = []
        self._t_wall    = []
        self._bad       = 0
        self._stop      = threading.Event()
        self._thread    = None

    # ── public ──

    def open(self):
        """Initialize I2C + sensor. Called by start() but exposed for tests."""
        rate = _resolve_refresh(self.refresh_hz)
        i2c = busio.I2C(board.SCL, board.SDA)
        self._mlx = adafruit_mlx90640.MLX90640(i2c)
        self._mlx.refresh_rate = rate

    def start(self):
        if self._thread is not None:
            raise RuntimeError("Mlx90640Capture already started")
        if self._mlx is None:
            self.open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop_and_collect(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        arr = (np.stack(self._frames) if self._frames
               else np.zeros((0, 24, 32), dtype=np.float32))
        return (
            arr,
            np.array(self._t_mono, dtype=np.int64),
            np.array(self._t_wall, dtype=np.float64),
            self._bad,
        )

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def bad_count(self) -> int:
        return self._bad

    # ── internal ──

    def _loop(self):
        while not self._stop.is_set():
            try:
                self._mlx.getFrame(self._buf)
                self._t_mono.append(time.monotonic_ns())
                self._t_wall.append(time.time())
                self._frames.append(
                    np.array(self._buf, dtype=np.float32).reshape(24, 32)
                )
            except (RuntimeError, ValueError):
                self._bad += 1


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--out', required=True, help='Output .npz path')
    p.add_argument('--duration', type=float, default=60.0,
                   help='Recording duration in seconds (default: 60)')
    p.add_argument('--refresh-hz', type=int, default=16,
                   choices=sorted(REFRESH_RATES_LAZY),
                   help='Sensor refresh rate; effective fps ≈ refresh/2 (default: 16)')
    args = p.parse_args()

    parent = os.path.dirname(args.out)
    if parent:
        os.makedirs(parent, exist_ok=True)

    print(f'[IR] Connecting to MLX90640 on I2C-1 ...')
    cap = Mlx90640Capture(refresh_hz=args.refresh_hz)
    cap.open()
    serial = ''.join(f'{x:04x}' for x in cap._mlx.serial_number)
    print(f'[IR] Sensor serial: {serial}  refresh_rate={args.refresh_hz}Hz')

    print(f'[IR] Recording {args.duration:.0f}s to {args.out} ...')
    t_start_mono = time.monotonic_ns()
    cap.start()
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print('\n[IR] Interrupted, flushing ...')
    frames, t_mono, t_wall, bad = cap.stop_and_collect()
    elapsed = (time.monotonic_ns() - t_start_mono) / 1e9

    np.savez(
        args.out,
        frames=frames,
        timestamps=t_wall,
        t_mono_ns=t_mono,
        refresh_rate_hz=args.refresh_hz,
        bad_frames=bad,
    )

    fps = len(frames) / elapsed if elapsed > 0 else 0.0
    print(f'[IR] Done. frames={len(frames)}  bad={bad}  '
          f'effective_fps={fps:.2f}  size={os.path.getsize(args.out)/1024:.0f}KB')


if __name__ == '__main__':
    main()
