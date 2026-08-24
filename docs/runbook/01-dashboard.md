# Web dashboard

## Start

```bash
saferoom-dashboard
```

It prints `SafeRoom dashboard ready — db=<path>`. Keep the terminal open;
`Ctrl+C` stops it. Use `--host` / `--port` to change the bind address.

## Open

- On the host itself: <http://localhost:8000/>
- From another device on the network: `http://<hostname>.local:8000/` or the
  host's IP.

## Telegram alerts (optional)

Export the credentials **before** starting the dashboard:

```bash
export SAFEROOM_TG_TOKEN=<bot-token>
export SAFEROOM_TG_CHAT_ID=<chat-id>
saferoom-dashboard
```

Only `fall_confirmed`, `fall_failopen`, `fall_fast` and `faint` send a Telegram
message. `fall_candidate` does not, because it is log-only.

Keep the token out of the repository. A gitignored `.local/telegram.env` that
you `source` before starting is the usual arrangement.

## Persistence

Events are stored in `dashboard.db` (SQLite), so history survives restarts of
the dashboard.

## Clearing history

The **Clear** button in the activity list empties the `events` table
permanently, without touching presence or the active alert. The equivalent from
a shell:

```bash
curl -X POST http://localhost:8000/events/clear
```

To start completely fresh, stop the dashboard and delete `dashboard.db`. The
next run creates an empty one.
