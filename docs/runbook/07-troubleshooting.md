# Diagnostics and maintenance

## No frames from the radar (drops, or 0 fps)

1. Press RST on the EVM and retry.
2. Check the USB ports:

   ```bash
   ls -l /dev/serial/by-id/
   ```

   Two links should appear, to `ttyUSB0` and `ttyUSB1`. If not, unplug and
   replug the EVM's USB cable.

3. Send `sensorStop` by hand if something is stuck:

   ```bash
   python -c "import serial,time; s=serial.Serial('/dev/ttyUSB0',115200,timeout=1); s.write(b'sensorStop\n'); time.sleep(0.5); print(s.read(s.in_waiting).decode())"
   ```

## IR will not calibrate ("hot pixels detected")

Leave the room and wait 60 s; the confirmer retries on its own. If it keeps
failing, look for something warm in the field of view: an incandescent bulb, a
laptop, or direct sunlight on a wall.

## Running the tests

```bash
pytest
```

53 tests should pass. A failure means something broke in a recent change.

## Updating the checkout

```bash
git pull --ff-only
```

If uncommitted local changes block the pull:

```bash
git stash && git pull --ff-only && git stash pop
```

## Installing or updating dependencies

```bash
pip install -e ".[viz,ml]"          # radar and ML environment
pip install -e ".[dashboard]"       # dashboard environment
pip install -e ".[viz,ml,dashboard,dev]"   # everything, single environment
```

## Orphaned dashboard processes

```bash
pgrep -f saferoom-dashboard     # list PIDs
pkill -f saferoom-dashboard     # kill them
```

## Inspecting persisted events

```bash
python -c "import sqlite3; c=sqlite3.connect('dashboard.db'); [print(r) for r in c.execute('SELECT ts,type,track_id,substr(details,1,80) FROM events ORDER BY id DESC LIMIT 10')]"
```

## Inspecting IR confirmer decisions

```bash
tail -5 logs/fall_events.jsonl
```

Each line is JSON with `event_type`, `payload.ir_decision`,
`payload.ir_features` and `payload.ir_rules`. This is how to work out why a
given fall came out `confirmed` rather than `candidate`.

## Sensor geometry drift

If tracks appear at the wrong height, check that `--sensor-height` and
`--sensor-tilt` match `sensorPosition` in the active `.cfg` and the physical
mount. `--z-offset` applies a runtime correction without editing either;
positive shifts up, negative shifts down.
