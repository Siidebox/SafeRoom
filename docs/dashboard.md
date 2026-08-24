# Dashboard

A single-page monitoring app for the SafeRoom fall-detection system. It runs on
the Raspberry Pi and is reachable from any device on the same network.

For day-to-day operation see [runbook/01-dashboard.md](runbook/01-dashboard.md).
This page covers how it is put together.

## Components

| Module | Purpose |
|--------|---------|
| `saferoom.dashboard.server` | FastAPI app: webhook receiver, SSE stream, Telegram bridge, serves the UI |
| `src/saferoom/dashboard/static/index.html` | The single-page UI (vanilla JS, no build step) |
| `saferoom.notify` | Small client used by `saferoom-read` to POST events |

## Install and run

```bash
pip install -e ".[dashboard]"
saferoom-dashboard
```

`--host`, `--port` and `--log-level` are available. Then open
`http://<host>:8000/`.

Events persist to `dashboard.db` (SQLite) in the working directory, so history
survives restarts.

## Telegram bridge (optional)

1. In Telegram, message **@BotFather**, send `/newbot`, follow the prompts.
2. Keep the **token** it gives you.
3. Send any message to your new bot.
4. Find your **chat id** at
   `https://api.telegram.org/bot<TOKEN>/getUpdates`, in `chat.id`.
5. Export both before launching the server:

```bash
export SAFEROOM_TG_TOKEN="<token>"
export SAFEROOM_TG_CHAT_ID="<chat-id>"
saferoom-dashboard
```

The token is a secret. Keep it in a gitignored file such as
`.local/telegram.env` and `source` it, or put it in a systemd `EnvironmentFile`.
Never commit it.

The bot sends a message on startup, and again for each critical event.

## Event types

| Type | Source | Severity | Sends Telegram? |
|------|--------|----------|-----------------|
| `presence` | radar | info | no |
| `presence_lost` | radar | info | no |
| `no_motion` | radar | warn | no |
| `fall_fast` | radar | critical | yes |
| `fall_confirmed` | radar / radar_ml | critical | yes |
| `fall_failopen` | radar | critical | yes |
| `fall_candidate` | radar | warn | no (log-only) |
| `faint` | radar | critical | yes |
| `heartbeat` | any | info | no |

`fall_candidate` is the IR confirmer's fail-closed veto. It is recorded but
never alerts; see [limitations.md](limitations.md).

## Publishing events from your own code

```python
from saferoom.notify import Notifier

notifier = Notifier(source="radar")
notifier.event("fall_fast", track_id=tid, vz=float(vz), max_z=float(max_z))
```

Periodically, from the main loop:

```python
notifier.event("heartbeat", fps=current_fps)
if presence_now and not presence_prev:
    notifier.event("presence", track_id=tid)
elif not presence_now and presence_prev:
    notifier.event("presence_lost")
```

`saferoom-read --dashboard http://localhost:8000` already does all of this.

## Running as a service

```ini
# /etc/systemd/system/saferoom-dashboard.service
[Unit]
Description=SafeRoom dashboard
After=network.target

[Service]
Type=simple
User=<user>
WorkingDirectory=/home/<user>/SafeRoom
EnvironmentFile=/home/<user>/.saferoom.env
ExecStart=/home/<user>/SafeRoom/.venv/bin/saferoom-dashboard
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now saferoom-dashboard
```
