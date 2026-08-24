# Recording dataset sessions

Synchronized radar + IR capture into a session directory. No GUI, no confirmer,
just raw data to be labelled afterwards.

The class targets, required variation and day schedule live in
[data_collection_protocol.md](../data_collection_protocol.md). This page is the
operational side: the commands, and how to tell a session is good.

## Before starting

1. Physical reset of the radar (RST button).
2. Confirm both ports exist: `/dev/ttyUSB0` (CLI) and `/dev/ttyUSB1` (data).
3. Check free disk space. A full capture day with IR runs to several GB.
4. At the start of each block (roughly 1 h), record a 60 s `calib` session with
   the room **empty**. That is the thermal background used by
   `saferoom-replay --ir-calib`. Ordinary sessions do not need an empty room;
   `saferoom-record` only records, it never calibrates.

Setting two shell variables saves repeating them:

```bash
SUBJ=<subject-id>
CFG=code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg
```

They live only in that terminal. Re-run them after reconnecting over SSH.

## Standard session (30 s)

```bash
saferoom-record \
  --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg "$CFG" \
  --duration 30 --ir-hz 16 \
  --name "${SUBJ}_fall_centre" --subject "$SUBJ" \
  --notes "front,cotton-shirt,afternoon"
```

`--name` should encode subject, activity and position; `--notes` should encode
orientation, clothing and time of day. Vary both across repetitions.

`--live-fall` is optional and informative: it runs the Tier-1 detector in the
same process and prints when a fall registers (`vz <= -1.15 m/s`). It opens no
window and barely affects the frame rate. Labels remain post-hoc either way, so
a session counts whether or not the detector fires.

## Block calibration session

```bash
saferoom-record \
  --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg "$CFG" \
  --duration 60 --ir-hz 16 \
  --name calib_b1 --subject none --notes "IR calibration, block 1"
```

## Long radar-only session

```bash
saferoom-record \
  --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg "$CFG" \
  --duration 60 --no-ir --name precheck_radar
```

## Validation

### A. Frame rates, after every session

The recorder prints:

```
[REC] Radar: 600 frames, 0 drops, 20.04 fps
[REC] IR: 240 frames, 0 bad, 7.92 fps
[REC] Manifest written: sessions/<id>/manifest.json
```

The session is valid when:

- `radar.fps_real >= 19.5`
- `thermal.fps_real >= 7.5`
- `radar.drops` under 1 % of frames
- `thermal.bad_frames` under 1 % of frames

Otherwise reset the radar and **re-record that session**. Also re-record any
take where the action itself went wrong, such as a half-fall or a hesitation.

### B. Calibration sessions, right after recording

```bash
python -c "from saferoom.evaluation.replay import load_background_from_session as l; l('sessions/<calib_id>'); print('calib OK')"
```

`calib OK` means the background is usable. An error means someone, or something
warm, was in the room. Re-record with the room empty.

### C. Radar-IR synchronization, roughly every 10 sessions

```bash
saferoom-check-sync sessions/<id> --max-drift-ms 50
```

This reports the drift between motion events detected in both streams. The
target is `max |dt| <= 50 ms`.

## Listing and cleanup

```bash
ls -lh sessions/
rm -r sessions/<id>      # remove one session
```

Sessions are gitignored and never committed.

Smoke-test and synthetic directories (`*_precheck`, `*_synccheck`, `synth_*`)
are not dataset material. Move them out of `sessions/` before training, and keep
`calib_*` separate as well, since it is background rather than labelled data.
