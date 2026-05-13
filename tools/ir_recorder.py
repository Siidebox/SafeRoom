"""
ir_recorder.py — Record MLX90640 thermal frames to a .npz file.

Designed to run on the Raspberry Pi 5 alongside `radar_reader.py` on the PC.
Each frame is paired with a Unix timestamp so radar and IR streams can be
aligned during offline labelling.

Usage
-----
    python tools/ir_recorder.py --out logs/ir_test.npz --duration 60

The output .npz contains:
    frames     : float32 array of shape (N, 24, 32), temperatures in Celsius
    timestamps : float64 array of shape (N,), Unix time at end of each frame
    refresh_rate_hz : scalar with the configured sensor refresh rate
    bad_frames : int, count of frames dropped due to read errors
"""

import argparse
import os
import sys
import time

import numpy as np

try:
    import board
    import busio
    import adafruit_mlx90640
except ImportError as e:
    sys.stderr.write(
        "Missing dependency: %s\n"
        "Install with: pip install adafruit-circuitpython-mlx90640 lgpio\n" % e
    )
    sys.exit(1)


REFRESH_RATES = {
    1:  adafruit_mlx90640.RefreshRate.REFRESH_1_HZ,
    2:  adafruit_mlx90640.RefreshRate.REFRESH_2_HZ,
    4:  adafruit_mlx90640.RefreshRate.REFRESH_4_HZ,
    8:  adafruit_mlx90640.RefreshRate.REFRESH_8_HZ,
    16: adafruit_mlx90640.RefreshRate.REFRESH_16_HZ,
    32: adafruit_mlx90640.RefreshRate.REFRESH_32_HZ,
}


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--out', required=True, help='Output .npz path')
    p.add_argument('--duration', type=float, default=60.0,
                   help='Recording duration in seconds (default: 60)')
    p.add_argument('--refresh-hz', type=int, default=4, choices=sorted(REFRESH_RATES),
                   help='Sensor refresh rate, halve for effective fps (default: 4)')
    args = p.parse_args()

    parent = os.path.dirname(args.out)
    if parent:
        os.makedirs(parent, exist_ok=True)

    print(f'[IR] Connecting to MLX90640 on I2C-1 ...')
    i2c = busio.I2C(board.SCL, board.SDA)
    mlx = adafruit_mlx90640.MLX90640(i2c)
    mlx.refresh_rate = REFRESH_RATES[args.refresh_hz]
    serial = ''.join(f'{x:04x}' for x in mlx.serial_number)
    print(f'[IR] Sensor serial: {serial}  refresh_rate={args.refresh_hz}Hz')

    buf = [0.0] * 768
    frames = []
    timestamps = []
    bad = 0

    t_start = time.time()
    t_end = t_start + args.duration
    print(f'[IR] Recording {args.duration:.0f}s to {args.out} ...')
    while time.time() < t_end:
        try:
            mlx.getFrame(buf)
            timestamps.append(time.time())
            frames.append(np.array(buf, dtype=np.float32).reshape(24, 32))
        except (RuntimeError, ValueError):
            bad += 1

    arr = np.stack(frames) if frames else np.zeros((0, 24, 32), dtype=np.float32)
    ts = np.array(timestamps, dtype=np.float64)
    np.savez(args.out, frames=arr, timestamps=ts,
             refresh_rate_hz=args.refresh_hz, bad_frames=bad)

    elapsed = time.time() - t_start
    fps = len(frames) / elapsed if elapsed > 0 else 0
    print(f'[IR] Done. frames={len(frames)}  bad={bad}  '
          f'effective_fps={fps:.2f}  size={os.path.getsize(args.out)/1024:.0f}KB')


if __name__ == '__main__':
    main()
