"""SafeRoom dashboard server.

Single-file FastAPI app that:
  * receives webhook events from radar_reader.py and (later) the MLX logger
  * persists events to SQLite (~/SafeRoom/dashboard.db)
  * streams live updates to the browser via Server-Sent Events
  * forwards critical events to a Telegram bot (if configured)
  * serves a single-page dashboard (templates/index.html)

Run on the Raspberry Pi:
    ~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py

Open from any LAN device:
    http://piSafeRoom.local:8000/

Environment (optional, for Telegram):
    SAFEROOM_TG_TOKEN     bot token from @BotFather
    SAFEROOM_TG_CHAT_ID   chat id to receive alerts
"""
from __future__ import annotations

import asyncio
import base64
import io
import json
import os
import sqlite3
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import httpx
import numpy as np
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

ROOT = Path.home() / "SafeRoom"
DB_PATH = ROOT / "dashboard.db"
TEMPLATES_DIR = Path(__file__).parent / "dashboard"
TG_TOKEN = os.environ.get("SAFEROOM_TG_TOKEN", "").strip()
TG_CHAT_ID = os.environ.get("SAFEROOM_TG_CHAT_ID", "").strip()
TG_CRITICAL_TYPES = {"fall_fast", "faint"}
SENSOR_TIMEOUT_S = 10.0

EVENT_TYPES = {"presence", "presence_lost", "no_motion", "fall_fast", "faint", "heartbeat"}


class EventIn(BaseModel):
    type: str = Field(..., description="event type, see EVENT_TYPES")
    timestamp: float = Field(default_factory=time.time)
    source: str = Field("radar", description="which sensor produced it")
    track_id: int | None = None
    details: dict[str, Any] = Field(default_factory=dict)


class ThermalIn(BaseModel):
    timestamp: float = Field(default_factory=time.time)
    width: int = 32
    height: int = 24
    pixels: list[float] = Field(..., description="row-major °C values, length=w*h")
    valid_pct: float = 100.0


def db_connect() -> sqlite3.Connection:
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(DB_PATH, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn


def db_init(conn: sqlite3.Connection) -> None:
    conn.executescript(
        """
        CREATE TABLE IF NOT EXISTS events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts REAL NOT NULL,
            type TEXT NOT NULL,
            source TEXT NOT NULL,
            track_id INTEGER,
            details TEXT NOT NULL DEFAULT '{}'
        );
        CREATE INDEX IF NOT EXISTS idx_events_ts ON events(ts DESC);
        """
    )
    conn.commit()


class State:
    def __init__(self) -> None:
        self.last_radar_ts: float = 0.0
        self.last_radar_fps: float = 0.0
        self.last_mlx_ts: float = 0.0
        self.last_mlx_fps: float = 0.0
        self.thermal_pixels: list[float] | None = None
        self.thermal_w = 32
        self.thermal_h = 24
        self.thermal_valid_pct = 100.0
        self.presence: bool = False
        self.last_presence_change: float = 0.0
        self.active_alert: dict[str, Any] | None = None  # current unresolved fall/faint
        self.subscribers: list[asyncio.Queue] = []

    async def broadcast(self, message: dict[str, Any]) -> None:
        dead: list[asyncio.Queue] = []
        for q in self.subscribers:
            try:
                q.put_nowait(message)
            except asyncio.QueueFull:
                dead.append(q)
        for q in dead:
            self.subscribers.remove(q)

    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        return {
            "now": now,
            "presence": self.presence,
            "last_presence_change": self.last_presence_change,
            "active_alert": self.active_alert,
            "radar": {
                "online": (now - self.last_radar_ts) < SENSOR_TIMEOUT_S,
                "fps": round(self.last_radar_fps, 1),
                "last_seen": self.last_radar_ts,
            },
            "mlx": {
                "online": (now - self.last_mlx_ts) < SENSOR_TIMEOUT_S,
                "fps": round(self.last_mlx_fps, 1),
                "last_seen": self.last_mlx_ts,
                "valid_pct": round(self.thermal_valid_pct, 1),
            },
            "thermal": {
                "width": self.thermal_w,
                "height": self.thermal_h,
                "pixels": self.thermal_pixels,
            } if self.thermal_pixels else None,
        }


state = State()


async def telegram_send(text: str) -> None:
    if not TG_TOKEN or not TG_CHAT_ID:
        return
    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            await client.post(
                f"https://api.telegram.org/bot{TG_TOKEN}/sendMessage",
                json={"chat_id": TG_CHAT_ID, "text": text, "parse_mode": "HTML"},
            )
    except Exception as e:  # noqa: BLE001
        print(f"telegram error: {e}")


@asynccontextmanager
async def lifespan(_: FastAPI):
    conn = db_connect()
    db_init(conn)
    app.state.db = conn
    print(f"SafeRoom dashboard ready — db={DB_PATH}")
    if TG_TOKEN and TG_CHAT_ID:
        await telegram_send("<b>SafeRoom</b> dashboard online.")
        print("telegram alerts: enabled")
    else:
        print("telegram alerts: DISABLED (set SAFEROOM_TG_TOKEN + SAFEROOM_TG_CHAT_ID)")
    yield
    conn.close()


app = FastAPI(title="SafeRoom dashboard", lifespan=lifespan)


@app.post("/event")
async def post_event(ev: EventIn, request: Request):
    if ev.type not in EVENT_TYPES:
        raise HTTPException(400, f"unknown type {ev.type!r}, must be one of {sorted(EVENT_TYPES)}")
    conn: sqlite3.Connection = request.app.state.db
    conn.execute(
        "INSERT INTO events (ts, type, source, track_id, details) VALUES (?, ?, ?, ?, ?)",
        (ev.timestamp, ev.type, ev.source, ev.track_id, json.dumps(ev.details)),
    )
    conn.commit()

    # Update derived state
    if ev.source == "radar":
        state.last_radar_ts = ev.timestamp
        state.last_radar_fps = float(ev.details.get("fps", state.last_radar_fps))
    if ev.type == "presence" and not state.presence:
        state.presence = True
        state.last_presence_change = ev.timestamp
    elif ev.type == "presence_lost" and state.presence:
        state.presence = False
        state.last_presence_change = ev.timestamp
    if ev.type in {"fall_fast", "faint"}:
        state.active_alert = {
            "type": ev.type,
            "timestamp": ev.timestamp,
            "track_id": ev.track_id,
            "details": ev.details,
        }
    if ev.type in TG_CRITICAL_TYPES:
        await telegram_send(
            f"🚨 <b>{ev.type.upper()}</b> detected at "
            f"{time.strftime('%H:%M:%S', time.localtime(ev.timestamp))}\n"
            f"details: <code>{json.dumps(ev.details)}</code>"
        )

    await state.broadcast({"kind": "event", "event": ev.model_dump(), "state": state.snapshot()})
    return {"ok": True}


@app.post("/thermal")
async def post_thermal(t: ThermalIn):
    expected = t.width * t.height
    if len(t.pixels) != expected:
        raise HTTPException(400, f"pixel count {len(t.pixels)} != {expected}")
    state.thermal_pixels = t.pixels
    state.thermal_w = t.width
    state.thermal_h = t.height
    state.thermal_valid_pct = t.valid_pct
    state.last_mlx_ts = t.timestamp
    # rolling fps estimate
    if not hasattr(state, "_mlx_times"):
        state._mlx_times = []  # type: ignore[attr-defined]
    state._mlx_times.append(t.timestamp)  # type: ignore[attr-defined]
    state._mlx_times[:] = [x for x in state._mlx_times if t.timestamp - x < 2.0]  # type: ignore[attr-defined]
    state.last_mlx_fps = len(state._mlx_times) / 2.0  # type: ignore[attr-defined]
    await state.broadcast({"kind": "thermal", "state": state.snapshot()})
    return {"ok": True}


@app.get("/state")
async def get_state():
    return JSONResponse(state.snapshot())


@app.get("/events")
async def get_events(request: Request, limit: int = 50):
    conn: sqlite3.Connection = request.app.state.db
    rows = conn.execute(
        "SELECT id, ts, type, source, track_id, details FROM events ORDER BY ts DESC LIMIT ?",
        (limit,),
    ).fetchall()
    return [
        {
            "id": r["id"],
            "ts": r["ts"],
            "type": r["type"],
            "source": r["source"],
            "track_id": r["track_id"],
            "details": json.loads(r["details"]),
        }
        for r in rows
    ]


@app.post("/ack_alert")
async def ack_alert():
    state.active_alert = None
    await state.broadcast({"kind": "ack", "state": state.snapshot()})
    return {"ok": True}


@app.get("/stream")
async def event_stream():
    queue: asyncio.Queue = asyncio.Queue(maxsize=128)
    state.subscribers.append(queue)

    async def gen():
        try:
            # initial state
            yield f"data: {json.dumps({'kind': 'snapshot', 'state': state.snapshot()})}\n\n"
            while True:
                try:
                    msg = await asyncio.wait_for(queue.get(), timeout=15.0)
                    yield f"data: {json.dumps(msg)}\n\n"
                except asyncio.TimeoutError:
                    yield ": keepalive\n\n"
        finally:
            if queue in state.subscribers:
                state.subscribers.remove(queue)

    return StreamingResponse(gen(), media_type="text/event-stream")


@app.get("/")
async def index():
    index_path = TEMPLATES_DIR / "index.html"
    if not index_path.exists():
        return HTMLResponse("<h1>dashboard template missing</h1>", status_code=500)
    return HTMLResponse(index_path.read_text(encoding="utf-8"))


if TEMPLATES_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(TEMPLATES_DIR)), name="static")


def main() -> None:
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


if __name__ == "__main__":
    main()
