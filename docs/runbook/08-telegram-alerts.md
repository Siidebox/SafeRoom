# Telegram alerts on the Raspberry Pi

What to set up so the dashboard sends fall and faint alerts to a phone. Do this
**once**; it survives reboots.

The bot token is a secret. It is **not** in the repository, and `git pull` will
not bring it to the Pi. You create it locally, once, in a gitignored file.

## 0. Prerequisites

- Repository cloned on the Pi and up to date.
- The dashboard extra installed, which provides `httpx`:

  ```bash
  pip install -e ".[dashboard]"
  ```

## 1. Create the bot and find the chat id

1. In Telegram, message **@BotFather** and send `/newbot`. Keep the token it
   returns.
2. Send any message to your new bot, so it is allowed to reply to you.
3. Visit `https://api.telegram.org/bot<TOKEN>/getUpdates` and read `chat.id`.

## 2. Store the credentials

Put them in a gitignored file on the Pi, readable only by you:

```bash
mkdir -p ~/SafeRoom/.local
cat > ~/SafeRoom/.local/telegram.env <<'EOF'
export SAFEROOM_TG_TOKEN="<bot-token>"
export SAFEROOM_TG_CHAT_ID="<chat-id>"
EOF
chmod 600 ~/SafeRoom/.local/telegram.env
```

`.local/` is in `.gitignore`, so this never reaches the repository.

Source it in any shell that will start the dashboard:

```bash
source ~/SafeRoom/.local/telegram.env
```

## 3. Verify the token independently

```bash
source ~/SafeRoom/.local/telegram.env
curl -s "https://api.telegram.org/bot${SAFEROOM_TG_TOKEN}/getMe" | grep -o '"username":"[^"]*"'
curl -s "https://api.telegram.org/bot${SAFEROOM_TG_TOKEN}/sendMessage" \
  -d chat_id="${SAFEROOM_TG_CHAT_ID}" \
  -d text="SafeRoom test $(date +%H:%M:%S)"
```

Expect your bot's username, `{"ok":true,...}`, and the message on your phone.

## 4. Full check through the dashboard

```bash
source ~/SafeRoom/.local/telegram.env
saferoom-dashboard
```

On startup you should see:

```
SafeRoom dashboard ready — db=<path>
telegram alerts: enabled
```

and the phone receives "SafeRoom dashboard online", which alone proves the
bridge works. `telegram alerts: DISABLED` means the environment variables were
not sourced in that terminal.

Dry-run a real alert from another terminal, with the dashboard running:

```bash
curl -X POST http://localhost:8000/event \
  -H 'Content-Type: application/json' \
  -d '{"type":"fall_fast","source":"radar","details":{"note":"dry-run"}}'
```

The phone receives a fall alert. Then clear the test:

```bash
curl -X POST http://localhost:8000/ack_alert
curl -X POST http://localhost:8000/events/clear
```

## Which events reach the phone

Only `fall_confirmed`, `fall_fast`, `fall_failopen` and `faint`
(`TG_CRITICAL_TYPES` in `saferoom.dashboard.server`). Presence events and
IR-vetoed `fall_candidate` events never notify.

## Checklist

- [ ] `~/SafeRoom/.local/telegram.env` exists, `chmod 600`, gitignored.
- [ ] `httpx` importable in the dashboard environment.
- [ ] `echo $SAFEROOM_TG_CHAT_ID` prints the chat id in a fresh shell after
      sourcing.
- [ ] Dashboard prints `telegram alerts: enabled` and the phone got the
      startup message.
- [ ] Dry-run `fall_fast` reached the phone; alert acknowledged and history
      cleared.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `telegram alerts: DISABLED` at startup | Variables not set in this shell. `source ~/SafeRoom/.local/telegram.env` before launching |
| `ModuleNotFoundError: httpx` | `pip install -e ".[dashboard]"` |
| `getMe` returns `{"ok":false, ... 401}` | Wrong or revoked token in `telegram.env` |
| `sendMessage` returns ok but nothing arrives | You never messaged the bot, or the chat id is wrong |
