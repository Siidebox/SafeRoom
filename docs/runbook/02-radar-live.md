# Live view: radar + IR + confirmer

On-screen 2D, 3D and IR panels with a fall counter. The confirmer runs in the
same process and emits `fall_confirmed`, `fall_failopen` or `fall_candidate` to
the dashboard.

## Prerequisites

1. **Physical reset** of the radar (RST button on the EVM).
2. Optionally, the dashboard running in another terminal, see
   [01-dashboard.md](01-dashboard.md). Without it everything still works, but
   events only reach `logs/fall_events.jsonl`.
3. The room **empty** for the first 30 seconds, for IR auto-calibration.

## Command

```bash
saferoom-read \
  --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
  --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
  --plot --plot3d --ir --ir-hz 16 --ir-rotate 90
```

This must run from a desktop terminal on the Pi, not over SSH, because it opens
a Qt window.

## Useful flags

| Flag | Purpose |
|---|---|
| `--plot` | 2D view: X-Y on top, X-Z from the side |
| `--plot3d` | Adds the OpenGL 3D panel |
| `--ir` | IR panel plus active confirmer |
| `--ir-hz 16` | MLX90640 refresh rate (about 8 effective fps) |
| `--ir-rotate {0,90,180,270}` | Compensates for the camera's mounting rotation |
| `--no-confirmer` | Disables the confirmer, leaving only rule-based `fall_fast`. Use for radar-only evaluation |
| `--ml-model models/fall_detector_xgb.pkl` | Runs the trained model alongside the rules |
| `--dashboard http://localhost:8000` | Posts events to the dashboard |

## What you should see at startup

```
[IR] live panel active @ 16 Hz, rotation 90
[IR] confirmer enabled — calibrating background...
```

Calibration finishes after about 30 s. If the room was not empty:

```
[IR] calibration aborted — hot pixels detected
```

Leave the room and wait 60 s; it retries on its own. If you keep walking back
in too early it never calibrates, the confirmer stays in `failopen`, and every
Tier-1 fall is reported as `fall_failopen`.

## What a fall looks like

- Console: `*** FALL DETECTED — Track N at (x, y, z) m ***`
- Milliseconds later the confirmer decides: `fall_confirmed`, `fall_failopen`
  or `fall_candidate`.
- The dashboard shows the row in its colour: red, red with a badge, or orange.
- `logs/fall_events.jsonl` gains a line with the full payload: the decision,
  the confidence, and which rules passed.

## Inspecting the event log

```bash
tail -5 logs/fall_events.jsonl
tail -1 logs/fall_events.jsonl | python -m json.tool
```
