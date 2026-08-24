# End-to-end live test

Validates the whole deployment on the Raspberry Pi:
**radar → `saferoom-read --ml-model` → detection → dashboard** (browser alert,
optionally Telegram). This checks that the trained model fires real alerts in
real time, not only in offline replay.

## 0. Prerequisites

- Trained model at `models/fall_detector_xgb.pkl`; if not, see
  [05-train-eval.md](05-train-eval.md).
- Radar flashed and connected; `/dev/ttyUSB0` (CLI) and `/dev/ttyUSB1` (data)
  both present.
- The dashboard extra installed (`pip install -e ".[dashboard]"`), in a separate
  environment if you keep them split.
- **Press RST** on the radar before starting.

## 1. Start the dashboard (terminal 1)

```bash
saferoom-dashboard
```

It prints `SafeRoom dashboard ready — db=<path>`. Leave the terminal open.
Open <http://localhost:8000/> on the Pi, or the host's IP from another device.

What the dashboard shows:

- **Live status card** — cycles through *no presence*, *all clear (presence)*,
  *fall detected*, *fall confirmed (IR)*, *immobility alert*. On a fall, press
  **Confirm** to acknowledge it.
- **RADAR card** — real fps and `online` while `saferoom-read` is publishing
  (heartbeat around 2 Hz). `0.0 fps` / `offline` means the radar is not
  publishing; see [section 5](#5-common-problems).
- **View room** — a modal with two panels. A **2D overhead plan** of the room
  (outline from the `.cfg`'s `boundaryBox`, the sensor as a triangle, one dot
  per tracked person with its TID, moving in real time and turning red during
  an active fall). And a **live thermal** panel, which only appears if the radar
  was started with `--ir`.
- **Recent activity** — event history coloured by type. **Clear** empties the
  history permanently; it does not affect presence or the active alert.

For Telegram alerts, export the credentials before starting the dashboard; see
[01-dashboard.md](01-dashboard.md).

## 2. Start the radar with the model (terminal 2)

Press **RST**, then run from the repository root so that the relative `models/`
and `logs/` paths resolve:

```bash
saferoom-read \
  --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
  --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
  --ml-model models/fall_detector_xgb.pkl \
  --dashboard http://localhost:8000 \
  --ir --ir-rotate 90
```

It should print:

```
[dashboard] posting fall/faint events to http://localhost:8000
[IR] capture active @ 16 Hz, rotation 90 → dashboard
```

The second line appears only with `--ir`. If the first line is missing, the
`--dashboard` flag did not reach the process.

### Variants

- **With the visualizer**: add `--plot`.
- **Thermal in the dashboard**: `--ir` works headless too; frames are forwarded
  at about 8 Hz. Keep `--ir-rotate` matched to the physical mount.
- **With IR fusion** (confirming or vetoing falls): needs `--plot --ir`. Headless
  runs have the confirmer disabled by design; plain `--ir` forwards thermal to
  the dashboard but performs no fusion.
- **Without the dashboard**: drop `--dashboard`. Console output and
  `logs/fall_events.jsonl` continue as normal.

## 3. Trigger a fall

Perform a controlled fall inside the covered area.

Console:

```
[ML] FALL DETECTED — Track 3 (p=0.98)
*** FALL DETECTED — Track 3 at (0.40, 2.10, 0.15) m ***
```

Event types you may see:

| Event | When | `source` | Critical alert? |
|---|---|---|---|
| `fall_confirmed` | The **ML model** detected it | `radar_ml` | Yes |
| `fall_fast` | Rule-based fall, no IR confirmer | `radar` | Yes |
| `fall_confirmed` | Rule-based fall, IR confirms (with `--ir`) | `radar` | Yes |
| `fall_failopen` | Rule-based fall, IR unavailable | `radar` | Yes |
| `fall_candidate` | Rule-based fall, IR **vetoes** it (fail-closed) | `radar` | No, log-only |
| `faint` | Immobility on the floor (Tier 2) | `radar` | Yes |

The model and the rules run **in parallel and independently**, so one fall can
produce both a `fall_confirmed` from the model (`source=radar_ml`) and a
`fall_fast` or `fall_confirmed` from the rules (`source=radar`). That is
expected.

## 4. Verify

```bash
curl -s http://localhost:8000/events?limit=10 | python -m json.tool
curl -s http://localhost:8000/state | python -m json.tool
```

Everything is also persisted to `dashboard.db` (SQLite) and appended to
`logs/fall_events.jsonl`, one JSON line per event with `event_type` and
`payload`.

Acknowledge the active alert from the UI, or with
`curl -X POST http://localhost:8000/ack_alert`.

## 5. Common problems

| Symptom | Cause and fix |
|---|---|
| No `[dashboard] posting ...` line | `--dashboard URL` missing from the command |
| Radar runs but the dashboard receives nothing | Is the dashboard running? Try `curl http://localhost:8000/state`. The notifier drops events when the server is down rather than blocking the radar |
| RADAR shows `0.0 fps` / `offline` while the radar runs | The position heartbeat is not arriving. Confirm `--dashboard` is set and that there is an active track; with nobody in the room no positions are sent |
| "View room" says *waiting for positions* | No active track yet (walk into the covered area), or `boundaryBox` is missing from the `.cfg` |
| Live thermal panel empty | Started without `--ir`, or the MLX90640 is not responding. Check for the `[IR] capture active` line, then check the sensor's I2C |
| `[WARN] Could not init dashboard notifier` | Malformed URL. Use `http://localhost:8000` with no trailing slash |
| `Could not load ML model` | Wrong `.pkl` path, or you ran from outside the repository root |
| IR fusion does nothing headless | By design. Use `--plot --ir` |
| Alert never reaches Telegram | Export `SAFEROOM_TG_TOKEN` and `SAFEROOM_TG_CHAT_ID` **before** starting the dashboard. `fall_candidate` never sends one |

More in [07-troubleshooting.md](07-troubleshooting.md).

## 6. Stopping

`Ctrl+C` in terminal 2 (the reader shuts the notifier down cleanly), then
`Ctrl+C` in terminal 1. History persists in `dashboard.db`.
