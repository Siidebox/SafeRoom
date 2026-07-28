# Telegram alerts on the Raspberry Pi — setup runbook

What to have ready on the Pi so the dashboard sends fall/faint alerts to your
phone. Do this **once**; then it survives reboots.

Bot: **@SafeRoom_ProjectBot** · chat id: `1298100567` (already known — no
`getUpdates` needed).

> The bot token is a secret. It is **not** in the repo and `git pull` will not
> bring it to the Pi. You create it locally, once, in a gitignored file.

---

## 0. Prerequisites (once)

- [ ] Repo cloned at `~/SafeRoom` and up to date:
  ```bash
  cd ~/SafeRoom && git pull
  ```
- [ ] Dashboard venv has `httpx` (the Telegram bridge needs it — the Pi failed
  on this before):
  ```bash
  ~/saferoom_cam/bin/python -c "import httpx; print('httpx', httpx.__version__)"
  # if it errors:
  ~/saferoom_cam/bin/pip install httpx
  ```
- [ ] Your phone has opened a chat with **@SafeRoom_ProjectBot** and sent it any
  message at least once (bots cannot message you first). Already done if you set
  the bot up — listed here only for a fresh phone.

## 1. Create the credentials file on the Pi (once)

`.local/` is gitignored, so this file lives only on the Pi. Paste the real
token where shown:

```bash
mkdir -p ~/SafeRoom/.local
cat > ~/SafeRoom/.local/telegram.env <<'EOF'
export SAFEROOM_TG_TOKEN=<paste-bot-token-from-BotFather>
export SAFEROOM_TG_CHAT_ID=1298100567
EOF
chmod 600 ~/SafeRoom/.local/telegram.env
```

Verify it is not tracked by git:

```bash
git -C ~/SafeRoom check-ignore .local/telegram.env   # must print the path
```

## 2. Make it automatic on every shell (once)

```bash
echo 'source ~/SafeRoom/.local/telegram.env' >> ~/.bashrc
source ~/.bashrc
echo "$SAFEROOM_TG_CHAT_ID"                    # must print 1298100567
```

## 3. Quick test — no radar, no dashboard needed

Confirms the token is valid and a message reaches your phone:

```bash
source ~/SafeRoom/.local/telegram.env
curl -s "https://api.telegram.org/bot${SAFEROOM_TG_TOKEN}/getMe" | grep -o '"username":"[^"]*"'
curl -s "https://api.telegram.org/bot${SAFEROOM_TG_TOKEN}/sendMessage" \
  -d chat_id="${SAFEROOM_TG_CHAT_ID}" \
  -d text="SafeRoom Pi test $(date +%H:%M:%S)"
```

Expect `"username":"SafeRoom_ProjectBot"`, `{"ok":true,...}`, and the message on
your phone.

## 4. Full check with the dashboard

```bash
source ~/SafeRoom/.local/telegram.env
~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
```

On startup you must see:

```
SafeRoom dashboard ready — db=/home/guillermo/SafeRoom/dashboard.db
telegram alerts: enabled
```

and your phone receives *"SafeRoom dashboard online."* — that alone proves the
bridge works. If instead it prints `telegram alerts: DISABLED`, the env vars
were not sourced in this terminal (redo step 2 or run `source` again).

Dry-run a real alert (dashboard running, in another terminal):

```bash
curl -X POST http://localhost:8000/event \
  -H 'Content-Type: application/json' \
  -d '{"type":"fall_fast","source":"radar","details":{"note":"pi dry-run"}}'
```

Phone receives `🚨 FALL_FAST detected at HH:MM:SS`. Then clear the test:

```bash
curl -X POST http://localhost:8000/ack_alert
curl -X POST http://localhost:8000/events/clear
```

---

## Checklist — ready to record

- [ ] `~/SafeRoom/.local/telegram.env` exists on the Pi, `chmod 600`, gitignored.
- [ ] `httpx` importable in the dashboard venv.
- [ ] `echo $SAFEROOM_TG_CHAT_ID` prints `1298100567` in a fresh shell.
- [ ] Dashboard prints `telegram alerts: enabled` and phone got "dashboard online".
- [ ] Dry-run `fall_fast` reached the phone; alert acked and history cleared.

## Which events reach the phone

Only `fall_confirmed`, `fall_fast`, `fall_failopen`, `faint`
(`dashboard_server.py` `TG_CRITICAL_TYPES`). Presence and IR-vetoed
`fall_candidate` events never notify.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `telegram alerts: DISABLED` at startup | env vars not in this shell → `source ~/SafeRoom/.local/telegram.env` before launching |
| `ModuleNotFoundError: httpx` | `~/saferoom_cam/bin/pip install httpx` |
| `getMe` returns `{"ok":false,...401}` | wrong/expired token in `telegram.env` |
| `sendMessage` ok but no message | phone never messaged the bot, or chat id wrong (must be `1298100567`) |
| Works in test, silent during demo | the event type isn't one of the 4 critical types (see above) |
