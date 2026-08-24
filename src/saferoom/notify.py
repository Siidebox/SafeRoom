"""SafeRoom dashboard notifier — minimal HTTP client for emitting events.

Used by radar_reader.py and mlx_logger.py to push events / thermal frames to
the dashboard server without blocking the producer loop.

Usage:
    from saferoom.notify import Notifier
    n = Notifier("http://localhost:8000")
    n.event("fall_fast", track_id=7, fps=20.1, vz=-1.5)
    n.thermal(frame_2d_celsius, valid_pct=98.0)
    n.shutdown()

If the dashboard is offline events are silently dropped after a short timeout
so the producer never stalls.
"""
from __future__ import annotations

import json
import os
import queue
import threading
import time
import urllib.error
import urllib.request
from typing import Any

import numpy as np

DEFAULT_URL = os.environ.get("SAFEROOM_DASHBOARD_URL", "http://localhost:8000")


class Notifier:
    def __init__(self, base_url: str = DEFAULT_URL, source: str = "radar"):
        self.base = base_url.rstrip("/")
        self.source = source
        self.q: queue.Queue[tuple[str, dict]] = queue.Queue(maxsize=256)
        self.stop_evt = threading.Event()
        self.t = threading.Thread(target=self._worker, daemon=True, name="saferoom-notifier")
        self.t.start()

    def event(self, type_: str, track_id: int | None = None,
              source: str | None = None, **details: Any) -> None:
        payload = {
            "type": type_,
            "timestamp": time.time(),
            "source": source or self.source,
            "track_id": track_id,
            "details": {k: _scalar(v) for k, v in details.items()},
        }
        self._enqueue("/event", payload)

    def tracks(self, fps: float, bounds, sensor_xy, tracks_list) -> None:
        """Push live track positions for the 2D room view.

        Not persisted server-side (like thermal) — only updates live state.
        `bounds` = (xmin,xmax,ymin,ymax,zmin,zmax) or None; `sensor_xy` = (x,y)
        or None; `tracks_list` = iterable of dicts with tid/x/y/z.
        """
        payload = {
            "timestamp": time.time(),
            "fps": float(fps),
            "bounds": list(bounds) if bounds is not None else None,
            "sensor": list(sensor_xy) if sensor_xy is not None else None,
            "tracks": [
                {
                    "tid": int(tk.get("tid", -1)),
                    "x": _scalar(tk.get("x", 0.0)),
                    "y": _scalar(tk.get("y", 0.0)),
                    "z": _scalar(tk.get("z", 0.0)),
                }
                for tk in tracks_list
            ],
        }
        self._enqueue("/tracks", payload)

    def thermal(self, frame_celsius, valid_pct: float = 100.0) -> None:
        arr = np.asarray(frame_celsius, dtype=np.float32)
        # Sanitise NaN/Inf to None so JSON is valid
        flat = [None if not np.isfinite(v) else float(v) for v in arr.ravel().tolist()]
        payload = {
            "timestamp": time.time(),
            "width": int(arr.shape[1]),
            "height": int(arr.shape[0]),
            "pixels": flat,
            "valid_pct": float(valid_pct),
        }
        self._enqueue("/thermal", payload)

    def shutdown(self) -> None:
        self.stop_evt.set()
        try:
            self.q.put_nowait(("__stop__", {}))
        except queue.Full:
            pass
        self.t.join(timeout=2.0)

    def _enqueue(self, path: str, payload: dict) -> None:
        try:
            self.q.put_nowait((path, payload))
        except queue.Full:
            pass  # drop on overflow rather than block the producer

    def _worker(self) -> None:
        while not self.stop_evt.is_set():
            try:
                path, payload = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            if path == "__stop__":
                return
            try:
                data = json.dumps(payload).encode("utf-8")
                req = urllib.request.Request(
                    self.base + path, data=data,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=1.5):
                    pass
            except (urllib.error.URLError, TimeoutError, ConnectionError):
                pass  # dashboard offline — drop event


def _scalar(v: Any) -> Any:
    if isinstance(v, (int, float, str, bool)) or v is None:
        return v
    try:
        return float(v)
    except (TypeError, ValueError):
        return str(v)
