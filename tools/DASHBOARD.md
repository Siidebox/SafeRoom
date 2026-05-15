# SafeRoom dashboard

Single-page monitoring app for the SafeRoom fall-detection system. Runs on
the Raspberry Pi and is reached from any device on the same LAN.

## Components

| File | Purpose |
|------|---------|
| `dashboard_server.py` | FastAPI server: webhook receiver, SSE stream, Telegram bridge, serves the HTML |
| `dashboard/index.html` | Single-page UI (Tailwind CDN, vanilla JS, no build step) |
| `saferoom_notifier.py` | Tiny client used by `radar_reader.py` and `mlx_logger.py` to POST events |

## Setup on the Raspberry Pi

```bash
~/saferoom_cam/bin/pip install fastapi 'uvicorn[standard]' httpx pydantic
```

(`numpy` is already in the venv from the MLX install.)

## Run

```bash
~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
```

Open from your phone / laptop on the same LAN:

```
http://piSafeRoom.local:8000/
http://<rpi-ip>:8000/
```

## Telegram bot (optional)

1. Open Telegram, search **@BotFather**, send `/newbot`, follow the prompts.
2. Save the **token** (e.g. `7891234567:AAH...`).
3. Send any message to your new bot.
4. Get your **chat id** by visiting
   `https://api.telegram.org/bot<TOKEN>/getUpdates` — look for `chat.id`.
5. Export both before launching the server:

```bash
export SAFEROOM_TG_TOKEN="7891234567:AAH..."
export SAFEROOM_TG_CHAT_ID="123456789"
~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
```

Or save them in `~/.saferoom.env` and source it from a systemd unit.

The bot will receive a message on startup and again whenever a `fall_fast`
or `faint` event fires.

## Wiring up producers

### radar_reader.py

Drop the notifier next to the existing `FallDetector` hookup. Add to the
imports:

```python
from saferoom_notifier import Notifier
notifier = Notifier(source="radar")
```

Inside the `FallDetector` callback (when a fall fires):

```python
notifier.event("fall_fast", track_id=tid, vz=float(vz), max_z=float(max_z))
```

Periodically (e.g. once per second in the main loop):

```python
notifier.event("heartbeat", fps=current_fps)
if presence_now and not presence_prev:
    notifier.event("presence", track_id=tid)
elif not presence_now and presence_prev:
    notifier.event("presence_lost")
```

### MLX logger (later, when integrated)

```python
notifier = Notifier(source="mlx")
# inside the capture loop, every N frames:
notifier.thermal(frame_clean_2d, valid_pct=valid_pct)
```

## Event types

| Type | Source | Severity | Sends Telegram? |
|------|--------|----------|------------------|
| `presence` | radar | info | no |
| `presence_lost` | radar | info | no |
| `no_motion` | radar | warn | no |
| `fall_fast` | radar | critical | yes |
| `faint` | radar | critical | yes |
| `heartbeat` | any | info | no |

## Running as a service (optional)

```ini
# /etc/systemd/system/saferoom-dashboard.service
[Unit]
Description=SafeRoom dashboard
After=network.target

[Service]
Type=simple
User=guillermo
EnvironmentFile=/home/guillermo/.saferoom.env
ExecStart=/home/guillermo/saferoom_cam/bin/python /home/guillermo/SafeRoom/tools/dashboard_server.py
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now saferoom-dashboard
```
