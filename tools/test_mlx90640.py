"""MLX90640 live thermal viewer — SafeRoom smoke test.

Run on the Raspberry Pi inside the saferoom_cam venv:
    ~/saferoom_cam/bin/python ~/SafeRoom/tools/test_mlx90640.py

Window controls:
    q / Esc   close
    s         save snapshot to ~/SafeRoom/logs/mlx_<timestamp>.png + raw .npy
    r         cycle refresh rate (2 / 4 / 8 / 16 / 32 Hz)
    c         cycle colormap (inferno / magma / plasma / hot / turbo / coolwarm)
    i         cycle interpolation (nearest / bilinear / bicubic)
    a         toggle frame averaging on/off (5-frame rolling mean)
    f         toggle fixed colour scale (auto-percentile <-> fixed 18-32 °C)
    m         toggle min/max markers
"""
from __future__ import annotations

import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import board
import busio
import adafruit_mlx90640
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

REFRESH_RATES = {
    2:  adafruit_mlx90640.RefreshRate.REFRESH_2_HZ,
    4:  adafruit_mlx90640.RefreshRate.REFRESH_4_HZ,
    8:  adafruit_mlx90640.RefreshRate.REFRESH_8_HZ,
    16: adafruit_mlx90640.RefreshRate.REFRESH_16_HZ,
    32: adafruit_mlx90640.RefreshRate.REFRESH_32_HZ,
}
CMAPS         = ["inferno", "magma", "plasma", "hot", "turbo", "coolwarm"]
INTERPS       = ["bilinear", "bicubic", "nearest"]
START_HZ      = 16    # SafeRoom default — best SNR/fps tradeoff for fall detection
AVG_WINDOW    = 5
FPS_WINDOW_S  = 2.0
T_MIN_VALID   = -20.0
T_MAX_VALID   = 80.0
FIXED_LO      = 18.0
FIXED_HI      = 32.0

OUT_DIR = Path.home() / "SafeRoom" / "logs"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def main() -> None:
    print("Initialising MLX90640...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=1_000_000)
    mlx = adafruit_mlx90640.MLX90640(i2c)
    print(f"  Serial: {[hex(x) for x in mlx.serial_number]}")

    state = {
        "hz": START_HZ,
        "cmap_idx": 0,
        "interp_idx": 0,
        "averaging": False,
        "fixed_scale": False,
        "show_markers": True,
        "frame_times": deque(),
        "buffer": deque(maxlen=AVG_WINDOW),
        "last_clean": None,
    }
    mlx.refresh_rate = REFRESH_RATES[state["hz"]]
    print(f"  Refresh rate: {state['hz']} Hz")

    fig, ax = plt.subplots(figsize=(10, 7))
    fig.canvas.manager.set_window_title("SafeRoom MLX90640 viewer")

    frame_buf = np.full((24, 32), np.nan, dtype=np.float32)
    img = ax.imshow(frame_buf, cmap=CMAPS[0], interpolation=INTERPS[0],
                    vmin=FIXED_LO, vmax=FIXED_HI, aspect="auto")
    fig.colorbar(img, ax=ax, label="°C")
    title = ax.set_title("starting...")
    ax.axis("off")
    min_marker, = ax.plot([], [], "o", mfc="none", mec="cyan",  ms=14, mew=2, visible=False)
    max_marker, = ax.plot([], [], "o", mfc="none", mec="red",   ms=14, mew=2, visible=False)

    raw = np.zeros(24 * 32, dtype=np.float32)

    def update(_):
        last_err = None
        for _attempt in range(6):
            try:
                mlx.getFrame(raw)
                last_err = None
                break
            except (ValueError, RuntimeError, OSError) as e:
                last_err = e
        if last_err is not None:
            title.set_text(f"frame error (skipped): {last_err}")
            return img, title, min_marker, max_marker

        frame = np.fliplr(raw.reshape(24, 32).copy())
        invalid = (frame < T_MIN_VALID) | (frame > T_MAX_VALID)
        bad_pct = 100.0 * invalid.sum() / invalid.size
        frame_clean = np.where(invalid, np.nan, frame)

        if np.isnan(frame_clean).all():
            title.set_text(f"all pixels invalid ({bad_pct:.0f}%) — check wiring / lower Hz")
            return img, title, min_marker, max_marker

        if state["averaging"]:
            state["buffer"].append(frame_clean)
            stacked = np.stack(state["buffer"])
            frame_clean = np.nanmean(stacked, axis=0)

        state["last_clean"] = frame_clean

        if state["fixed_scale"]:
            lo, hi = FIXED_LO, FIXED_HI
        else:
            lo = float(np.nanpercentile(frame_clean, 5))
            hi = float(np.nanpercentile(frame_clean, 95))
            if hi - lo < 2:
                hi = lo + 2

        img.set_data(frame_clean)
        img.set_clim(lo, hi)

        if state["show_markers"]:
            valid = ~np.isnan(frame_clean)
            if valid.any():
                flat = np.where(valid, frame_clean, np.nan)
                ymax, xmax = np.unravel_index(np.nanargmax(flat), flat.shape)
                ymin, xmin = np.unravel_index(np.nanargmin(flat), flat.shape)
                max_marker.set_data([xmax], [ymax])
                min_marker.set_data([xmin], [ymin])
                max_marker.set_visible(True)
                min_marker.set_visible(True)
        else:
            min_marker.set_visible(False)
            max_marker.set_visible(False)

        now = time.time()
        state["frame_times"].append(now)
        while state["frame_times"] and now - state["frame_times"][0] > FPS_WINDOW_S:
            state["frame_times"].popleft()
        fps = len(state["frame_times"]) / FPS_WINDOW_S

        flags = ("AVG " if state["averaging"] else "    ") + \
                ("FIX " if state["fixed_scale"] else "AUTO")
        title.set_text(
            f"{state['hz']} Hz | {fps:4.1f} fps | valid {100-bad_pct:3.0f}% | "
            f"{flags} | {INTERPS[state['interp_idx']]} | {CMAPS[state['cmap_idx']]} | "
            f"p5={lo:.1f}  p95={hi:.1f}  mean={np.nanmean(frame_clean):.1f} °C"
        )
        return img, title, min_marker, max_marker

    def on_key(event):
        k = event.key
        if k in ("q", "escape"):
            plt.close(fig)
            return
        if k == "s":
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            png_path = OUT_DIR / f"mlx_{ts}.png"
            fig.savefig(png_path, dpi=150, bbox_inches="tight")
            if state["last_clean"] is not None:
                np.save(OUT_DIR / f"mlx_{ts}.npy", state["last_clean"])
            print(f"saved {png_path} (+ .npy)")
        elif k == "r":
            opts = sorted(REFRESH_RATES.keys())
            state["hz"] = opts[(opts.index(state["hz"]) + 1) % len(opts)]
            mlx.refresh_rate = REFRESH_RATES[state["hz"]]
            state["frame_times"].clear()
            state["buffer"].clear()
            print(f"refresh rate -> {state['hz']} Hz")
        elif k == "c":
            state["cmap_idx"] = (state["cmap_idx"] + 1) % len(CMAPS)
            img.set_cmap(CMAPS[state["cmap_idx"]])
            print(f"colormap -> {CMAPS[state['cmap_idx']]}")
        elif k == "i":
            state["interp_idx"] = (state["interp_idx"] + 1) % len(INTERPS)
            img.set_interpolation(INTERPS[state["interp_idx"]])
            print(f"interpolation -> {INTERPS[state['interp_idx']]}")
        elif k == "a":
            state["averaging"] = not state["averaging"]
            state["buffer"].clear()
            print(f"averaging -> {state['averaging']}")
        elif k == "f":
            state["fixed_scale"] = not state["fixed_scale"]
            print(f"fixed_scale -> {state['fixed_scale']}")
        elif k == "m":
            state["show_markers"] = not state["show_markers"]
            print(f"show_markers -> {state['show_markers']}")

    fig.canvas.mpl_connect("key_press_event", on_key)
    anim = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
    plt.show()
    _ = anim  # keep reference alive until window closes


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
