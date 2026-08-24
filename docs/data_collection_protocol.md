# Data collection protocol

Version 1.1 (2026-07-08). For use once the v2 enclosure is assembled, with the
IWR6843AOPEVM radar and the MLX90640 thermal camera connected to the
Raspberry Pi 5.

Changes in v1.1: adds the **minimum viable protocol** (one subject, one day) as
the target for the first capture day, unscripted negative sessions with
confounders, continuous-scenario sessions, and the `fall_lying` label (see
`labeling_protocol.md` v1.1). The full v1.0 breakdown becomes the extended
target if more days or subjects become available.

## Goal

Produce a labelled, **time-synchronized multimodal** dataset (radar + IR) good
enough to train and rigorously evaluate the fall detection model. The primary
model is radar-only; IR is always recorded as visual ground truth and as a
secondary baseline.

## Minimum viable protocol (first day, one subject)

Enough for an honest results chapter. Listed highest priority first; if the day
runs short, cut from the bottom.

| Priority | Class | Repetitions | Notes |
|---|---|---|---|
| 1 | `fall` | 25-30 | 4 orientations (front, left, right, backward), at least 3 positions. Use a crash mat. |
| 2 | **unscripted negatives** | 2 sessions x 15-30 min | See below. No script, with confounders. |
| 3 | `near_fall` | 15 | |
| 4 | `sit` | 20 | Half on a chair, half on the floor. |
| 5 | `walk` | 15-20 | Include trajectories perpendicular to the beam. |
| 6 | `lie` | 10 | |
| 7 | `stand` | 10 | |
| 8 | `none` | 2 x 100 s | |
| 9 | **continuous scenario** | 2 sessions x ~5 min | See below. |

**Unscripted negative sessions (priority 2).** Ordinary life in the room with no
falls, deliberately including confounders that can trigger radar false
positives: dropping a backpack or other heavy object, crouching to pick
something off the floor, sitting down abruptly, opening and closing the door,
moving a chair, stretching out on the floor to reach under furniture.

These sessions are what give statistical power to the false-alarms-per-hour
metric and to the radar+IR fusion study. Without them, with only short scripted
activities, the radar can score zero false positives and the comparison says
nothing. Label them with their ordinary classes (`walk`, `sit`, `none`, ...).

**Continuous-scenario sessions (priority 9).** One session containing a mixed
sequence: enter, walk, stand, sit, walk, fall, stay on the floor for at least
10 s, get up, leave. These serve as a realistic validation set. Everywhere else
in the dataset each session holds a single class, which makes the LOSO folds
degenerate: the `fall` fold has almost no negatives.

**Single-subject bias.** With one subject, validation is leave-one-session-out,
not leave-one-subject-out, so generalization claims are limited to "unseen
sessions from the same subject and environment". Record the bias in
`manifest.notes` and in the limitations section. A second subject would repeat
blocks 1-5 at half the repetitions.

## Physical setup

| Item | Target | Notes |
|---|---|---|
| Sensor height | 2.04 m | Must match `sensorPosition` in the active `.cfg` |
| Tilt | 10 deg downward | Validated 2026-04-20 |
| Room | 5.10 x 3.7 m | `boundaryBox` in the `.cfg`; X is asymmetric, sensor 1.30 m from the corner |
| Floor | Clear, no clutter above 0.5 m | Avoids static reflectors |
| MLX90640 | Beside the radar, FOV aligned | Confirm 16 Hz refresh |
| Lighting | Irrelevant for IR | Natural light helps if shooting reference video |

Before recording: the Pi is powered and booted, `sensorStop` and `flushCfg` have
been sent (each session sends its own `.cfg` at the start), and a quick check
has been run with `saferoom-record --duration 10 --name precheck`, confirming
`radar.fps_real >= 19.5` and `thermal.fps_real >= 7.5`.

## Output layout

Each session produces one directory:

```
sessions/<YYYYMMDD_HHMMSS>_<subject>_<activity>/
├── radar.csv      # MlCsvLogger, 35 columns, includes t_mono_ns
├── thermal.npz    # MLX90640 24x32 float32 + t_mono_ns + t_wall
└── manifest.json  # metadata, plus LabelSpans after labelling
```

## Extended class targets

Minimums before the dataset can be considered trainable.

| Class | Operational definition | Target repetitions |
|---|---|---|
| `fall` | Full fall from standing to the floor, not getting up for at least 3 s. Front, left, right, backward, roughly 25 % each. | 60 |
| `near_fall` | Trip or loss of balance recovered before touching the floor. | 30 |
| `sit` | Sitting down under control, on the floor or a chair. | 60 |
| `lie` | Voluntarily lying down, on a bed or mat. | 30 |
| `walk` | Walking around the room in various directions. | 60 |
| `stand` | Standing still in various positions. | 30 |
| `none` | Empty room. | 600 s total (about 6 sessions x 100 s) |

Estimated total: ~35 min of activity plus ~10 min empty room, so **45 min** net.

## Required variation

For the model to generalize, each class must span:

- **Position** in the room: at least 4 points (corners plus centre).
- **Orientation** relative to the radar: 0, 45, 90, 135 and 180 degrees.
- **Subjects**: ideally 2 or more, with different heights and builds.
- **Clothing**: at least one absorbent outfit (thick cotton) and one reflective
  one (synthetic, jacket).
- **Time of day**: at least 2 different times. IR is sensitive to ambient
  temperature and sunlight.

If two subjects are not available, record that bias in each session's
`manifest.notes` and in the limitations section.

## Per-session procedure

A session is one continuous recording of **a single activity**. Do not mix
classes in one session except for the specific continuous-scenario tests.

1. **Decide** class, subject, position, orientation.

2. **Start recording:**

   ```bash
   saferoom-record \
     --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
     --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
     --duration 30 --ir-hz 16 \
     --name <subject>_<activity>_<position> \
     --subject <subject> \
     --notes "<orientation>,<clothing>,<time>"
   ```

   `saferoom-record` only records; it does **not** calibrate the IR. That is
   done live by `IrConfirmer`. For offline analysis the thermal background comes
   from a dedicated `calib` session: at the start of each recording block
   (roughly 1 h), record 60 s with the room **empty**:

   ```bash
   saferoom-record --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
     --cfg ... --duration 60 --ir-hz 16 --name calib_blockN --subject none \
     --notes "IR calibration, block N"
   ```

   Then `saferoom-replay --ir-calib sessions/<calib_id>` builds the thermal
   background from it. If anyone was in the room during calibration, replay
   rejects it with a clear error and it must be re-recorded. Ordinary sessions
   do **not** need to start with an empty room.

3. **Perform** the action following these timings:
   - Falls: stand for 5 s, execute, then **stay down for at least 8 s** so that
     `fall_lying` covers the IR confirmer's 5 s window.
   - Near-falls: stand for 5 s, execute the stumble, recover.
   - Sit / lie: stand 3 s, sit or lie down, stay still 10 s.
   - Walk: walk continuously for 30 s.
   - Stand: stand still; change position every 10 s between sessions.
   - None: nobody in the room for the whole session.

4. **Verify.** The recorder prints `radar.fps_real` and `thermal.fps_real`.
   Re-record if `radar.fps_real < 19.5`, `thermal.fps_real < 7.5`,
   `radar.drops > 1 %` of frames, or `thermal.bad_frames > 1 %` of frames.

5. **Label afterwards**, not during the session:

   ```bash
   saferoom-label-mm sessions/<id>/
   ```

## Suggested capture-day schedule

Each block starts with a **60 s `calib` session in an empty room**, providing
the thermal background for `saferoom-replay --ir-calib`.

| Block | Duration | Content |
|---|---|---|
| Setup + precheck | 45 min | Mount the sensor (2.04 m, 10 deg), cable to the Pi, smoke test |
| Block 1 | 1 h | calib 60 s, then 25-30 falls (4 orientations x at least 3 positions) |
| Break | 15 min | |
| Block 2 | 30-45 min | calib 60 s, then the first long unscripted negative session |
| Block 3 | 30 min | calib 60 s, then 15 near_fall and 20 sit |
| Block 4 | 30 min | 15-20 walk, 10 lie, 10 stand, 2 x none 100 s |
| Block 5 | 30 min | calib 60 s, second negative session, 2 continuous scenarios |
| Labelling | 1-2 h | All of the day's sessions; can be a different day |

With a second subject, repeat blocks 1-3 at half the repetitions on another day.

## Pre-capture checklist

1. Sensor mounted at **2.04 m, 10 deg tilt**, in the same position as
   `sensorPosition` in the active `.cfg`.
2. MLX90640 beside the radar with FOV aligned. **Write down the mounting
   rotation** (0/90/180/270 deg); the same value must be passed to
   `saferoom-read --ir-rotate` live and `saferoom-replay --ir-rotate` offline.
3. Pi booted, radar on `/dev/ttyUSB0` (CLI) and `/dev/ttyUSB1` (data).
4. Smoke test: `saferoom-record --duration 10 --name precheck`, confirming
   `radar.fps_real >= 19.5` and `thermal.fps_real >= 7.5`.
5. Visual check with `saferoom-read --plot`: walk around and confirm the track
   follows the person inside the `boundaryBox`, and that `maxZ` is close to the
   expected standing height.
6. Record block 1's `calib` session (60 s, empty room) and validate it on the
   spot:

   ```bash
   python -c "from saferoom.evaluation.replay import load_background_from_session as l; l('sessions/<calib_id>'); print('calib OK')"
   ```

   If someone was in the room it fails with the reason. Re-record.
7. Rehearse one fall onto the mat with `--plot` and confirm Tier 1 fires, or
   note that it did not so the threshold can be reviewed.
8. Crash mat in place; informed consent signed by the subject. Even when the
   subject is the author, keep a written record.

## Minimum dataset quality

After labelling, check that:

- Every class meets the target repetitions above.
- No session is the only instance of its class for a given subject, which is
  required for LOSO-CV with cross-subject generalization.
- The duration distribution (`manifest.duration_s`) has no extreme outliers,
  and no fall session is shorter than 5 s.

Recorded sessions are not committed to this repository (`sessions/` is
gitignored); keep them in the project's data store, and never publish video
without consent. The default outputs are CSV and NPZ only.

## Limitations to carry into the write-up

- Lateral movement perpendicular to the beam produces low Doppler.
- `maxZ - minZ` is the cluster's vertical extent, not the person's stature.
- IR does not label anything automatically. Labelling remains manual and depends
  on the operator's judgement (see `labeling_protocol.md`).
- 16 Hz IR gives about 8 effective fps through subpaging, so IR temporal
  resolution is roughly 125 ms.
- Without genuine subject diversity, the models will overfit to the operator.

See [limitations.md](limitations.md) for the full list.
