# SafeRoom — Demo Recording Script

Two deliverables come out of this document:

- **Video 1 — The demo (~6–8 min final cut).** Recorded in the real room. Phone camera films the room; the dashboard is screen-recorded on the laptop in parallel; a phone insert shows the Telegram alert. Voice-over (VO) is recorded afterwards against the edited picture.
- **Video 2 — The explanation (~5 min, standalone).** Guillermo on camera (or over screenshots) explaining what the demo showed and how the system works. Independent of the 15-min slide presentation.

Both scripts are in English. Target speaking pace ~140–150 words per minute.

---

## 0. System configuration used for the demo

The demo runs the **radar-only ML detector** — the primary contribution of the thesis. IR fusion stays **off** (it is reported as a rejected negative result), but the thermal camera still streams to the dashboard's Room view because headless `--ir` forwards frames without enabling the confirmer.

| Component | Setting |
|---|---|
| Detector | XGBoost `models/fall_detector_xgb.pkl` + rule baseline (both run in parallel) |
| IR | `--ir --ir-rotate 90` headless → thermal view only, **no fusion** |
| Dashboard | On the Pi, port 8000; viewed + screen-recorded from the laptop |
| Dashboard language | English (default; `EN` button in the header toggles) |
| Telegram | Enabled (setup in §2) |

> **Label note.** The ML detector's event type is `fall_confirmed`, whose
> English UI strings used to read "FALL CONFIRMED (IR)". Since this demo runs
> no IR fusion, the "(IR)" suffix was removed from the two English strings
> (`status_alert_fall_confirmed` / `ev_fall_confirmed` in
> `tools/dashboard/index.html`) — **applied 2026-07-23**. The Spanish strings
> keep "(IR)"; record with the UI in English. Remember to `git pull` on the
> Pi before recording day so it serves the updated UI.

---

## 1. Pre-flight checklist (day of recording)

Work through this top to bottom. Everything must be green before the first take.

### 1.1 Hardware and services

- [ ] Pi powered, radar connected, `/dev/ttyUSB0` and `/dev/ttyUSB1` present.
- [ ] **Press RST on the radar** (it holds bad state between sessions).
- [ ] Telegram env vars exported in Terminal 1 **before** starting the dashboard (§2).
- [ ] **Terminal 1 — dashboard:**

  ```bash
  export SAFEROOM_TG_TOKEN=<token>
  export SAFEROOM_TG_CHAT_ID=<chat-id>
  ~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
  ```

  Expect: `SafeRoom dashboard ready — db=/home/guillermo/SafeRoom/dashboard.db`
  and `telegram alerts: enabled`. Your phone receives *"SafeRoom dashboard online."*

- [ ] **Terminal 2 — radar** (from the repo root; press RST first):

  ```bash
  cd ~/SafeRoom
  .venv/bin/python tools/radar_reader.py \
    --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
    --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
    --ml-model models/fall_detector_xgb.pkl \
    --dashboard http://localhost:8000 \
    --ir --ir-rotate 90
  ```

  Expect both startup lines:

  ```
  [dashboard] posting fall/faint events to http://localhost:8000
  [IR] capture active @ 16 Hz, rotation 90° → dashboard
  ```

- [ ] Laptop browser at `http://piSafeRoom.local:8000/` (or the Tailscale IP).
  Language set to **EN**. Radar card shows `online` with real FPS.
- [ ] **Send the §2 Telegram dry-run curl once now** — confirms internet +
  bot reachability on recording day, before any take depends on it.
- [ ] Laptop: **disable display sleep and all notifications** (the scene-5 wait
  is ~40 s of hands-off screen recording).
- [ ] Walk into the room once: dot appears in **Room view**, thermal blob visible
  in **Live thermal**. Fall onto the mattress once as a full rehearsal: alert +
  Telegram message arrive. **While lying on the mattress, check the reader
  console: maxZ must read clearly below 0.80 m** (the faint ceiling). If it
  doesn't, use a thinner mat for scene 5. Then **Acknowledge** and **Clear**.
- [ ] Before every clap: glance at the dashboard — radar fps ticking and the
  page live (an SSE/browser disconnect freezes it silently).

### 1.2 Clean state between takes

```bash
# clear the activity list (permanent):
curl -X POST http://localhost:8000/events/clear
# dismiss a stuck active alert:
curl -X POST http://localhost:8000/ack_alert
```

For a completely fresh database before the first take: stop the dashboard,
`rm ~/SafeRoom/dashboard.db`, start it again.

### 1.3 Recording setup

- [ ] **Phone on tripod** framing the whole room, landscape, 1080p or better.
  Lock exposure/focus so the image doesn't pump when you move.
- [ ] **Laptop screen recording** of the browser, full-screen (F11), dashboard
  only — no other tabs or notifications. OBS or the OS recorder, 30 fps.
- [ ] **Second phone (or the same one afterwards) for the Telegram insert** —
  see scene 4 notes; the Telegram shot can be captured separately.
- [ ] **Sync clap:** start both recordings, stand in frame, clap once loudly.
  The audio spike + visible clap aligns room footage and screen capture in the
  editor. Do this at the start of *every* take.
- [ ] **Safety:** thick mattress or gym mat in the fall zone. Clear obstacles.
  Falls are onto the mattress only — do not fall on bare floor.
- [ ] One scenario per take, named `takeNN_<scenario>.mp4` (e.g.
  `take03_fall_ml.mp4`). Clear history between takes so each starts clean.

### 1.4 Where to fall

Perform the main fall in the **well-covered zone** (center / near side of the
room, moving toward or away from the sensor, i.e. with a radial velocity
component). The optional bonus fall (scene 6) is on the **lateral side**, where
Doppler is weakest — that one showcases the ML model closing the lateral gap.

**Track warm-up rule (applies to every fall take):** the ML detector needs a
full 30-frame (1.5 s) buffer on your track before it can score, and a track
re-spawn (new TID) empties it. Before falling, confirm on Room view that your
dot has kept the **same track ID for at least ~2 s**; if the dot flickers or
the TID changes, walk a small loop and try again.

---

## 2. Telegram setup + test (first time only)

The dashboard already contains the Telegram bridge; it only needs a bot token
and a chat id in the environment. Do this once, days before recording.

**Step 1 — create the bot.** In the Telegram app, open a chat with
**@BotFather** → send `/newbot` → choose a display name (e.g. *SafeRoom
Alerts*) and a unique username ending in `bot` (e.g. `saferoom_alerts_bot`).
BotFather replies with the **HTTP API token** — a string like
`1234567890:AAF...xyz`. Keep it private (do not commit it, do not show it on
camera).

**Step 2 — get your chat id.** Open a chat with your new bot and send it any
message (e.g. "hi") — required, bots cannot message you first. Then, from any
machine:

```bash
curl -s "https://api.telegram.org/bot<TOKEN>/getUpdates"
```

In the JSON reply, find `"chat":{"id":123456789,...}` — that number is your
`SAFEROOM_TG_CHAT_ID`. (If the reply is empty, send the bot another message
and re-run.)

**Step 3 — configure the dashboard.** In the terminal where you launch the
dashboard, **before** launching it:

```bash
export SAFEROOM_TG_TOKEN=1234567890:AAF...xyz
export SAFEROOM_TG_CHAT_ID=123456789
~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
```

Startup must print `telegram alerts: enabled`, and the phone immediately
receives *"SafeRoom dashboard online."* — that alone proves the bridge works.
To make it permanent, append the two `export` lines to `~/.bashrc` on the Pi.

**Step 4 — dry-run an alert without the radar.** With the dashboard running:

```bash
curl -X POST http://localhost:8000/event \
  -H 'Content-Type: application/json' \
  -d '{"type":"fall_fast","source":"radar","details":{"note":"telegram dry-run"}}'
```

Within a couple of seconds the phone receives `🚨 FALL_FAST detected at
HH:MM:SS`. The dashboard also flips to **FALL DETECTED** (this injected event
is indistinguishable from a real one) — press **Acknowledge** and **Clear** the
history afterwards so the test doesn't pollute the demo.

**What triggers Telegram:** only `fall_confirmed`, `fall_fast`,
`fall_failopen`, and `faint`. Presence events and IR-vetoed `fall_candidate`
events never do.

---

## 3. Video 1 — Demo shot list and voice-over script

Six scenes plus an optional bonus. Raw material ≈ 8–9 min; trim to 6–8 in the
edit. Each scene: **[ACTION]** what happens on camera, **[DASHBOARD]** what the
screen recording must show, **[VO]** the narration to record afterwards.

Suggested edit layout: room footage full-frame, dashboard as a large
picture-in-picture; cut the dashboard to full-frame at every alert moment.

### Scene 1 — Cold open: the empty room (~45 s)

**[ACTION]** Empty room. Slow phone pan: the room, then a close-up of the
radar sensor on the wall, then the Raspberry Pi. Nobody in frame.

**[DASHBOARD]** Status card: **No presence — Room appears empty**. Radar card
`online` with ~20 fps. Open **Room view** briefly: floor plan drawn, no dots,
thermal panel showing the cold room.

**[VO]**
> "This is a real bedroom in a real house — five point one by three point seven
> meters. On the wall, at two meters high, there is a single sixty-gigahertz
> millimeter-wave radar, and next to it a Raspberry Pi that runs the entire
> system. There is no video camera watching this room. What you see on the right is
> the live dashboard: right now it reports 'no presence', the radar is online
> at twenty frames per second, and the floor plan is empty. Everything in this
> demo is running live, on the Pi, with no cloud service involved."

### Scene 2 — Entry, presence, and tracking (~60 s)

**[ACTION]** Walk into the room at normal pace. Walk a defined path: toward
the far wall, pause, back toward the sensor, pause, then across the room once.
Keep movements natural.

**[DASHBOARD]** Status flips to **All clear — Presence detected**; a
`Presence detected` event appears in Recent activity. In **Room view**, a dot
with a track ID appears and follows the walk in real time; the thermal panel
shows a warm blob moving.

**[VO]**
> "The moment I walk in, the system detects presence and starts tracking me.
> The dot on the floor plan is me — it is not video, it is the position of a
> cluster of radar reflections, updated twenty times per second. Below it, the
> thermal view: a thirty-two by twenty-four pixel infrared array. At that
> resolution I am just a warm blob — neither sensor can identify who I am, and
> that is the point. This is what privacy-preserving monitoring looks like:
> the system knows where I am and how I move, and nothing more."

### Scene 3 — Everyday activity, no false alarms (~75 s)

**[ACTION]** In sequence, with ~5 s pauses: (1) sit down on a chair, stand
up; (2) crouch to pick something off the floor, stand up; (3) sit on the edge
of the bed, lie down on the bed for a few seconds, sit up, stand. Keep every
movement at natural speed — do not exaggerate slowness.

**[DASHBOARD]** Status stays **All clear** throughout. No critical alert, no
red dot, phone silent.

**[VO]**
> "A fall detector is only useful if it stays quiet the rest of the day. These
> are the movements that fool naive systems: sitting down, crouching, lying on
> the bed. Each one ends with a rapid change in height — exactly what a fall
> looks like at first glance. The system runs two detectors in parallel: a
> hand-tuned rule baseline, and a machine-learning model that scores a
> one-and-a-half-second window of thirty-six motion and shape features. In a
> replay of every recorded session, the rule baseline raised about ten false
> alarms per hour; the learned model raised none on the same recordings. As
> you can see — the dashboard stays green, and my phone stays silent."

**Contingency:** if the rule baseline *does* fire a false `fall_fast` during
this scene, keep the take — it is the thesis result on camera. Alternative VO
ending:
> "...and here you can see it live: the rule baseline just fired a false
> alarm on an ordinary movement — the learned model, correctly, did not.
> This is exactly why the ML detector replaces the rules."

(Then Acknowledge, clear history, and continue.)

**If the *ML* detector fires during this scene** (event with source
`radar_ml`): stop, Acknowledge, clear history, and retake — the scene's VO
depends on the ML staying quiet, so do not narrate over it. Note the event
for the thesis error analysis.

### Scene 4 — The fall (~60 s)

**[ACTION]** Walk to the mattress in the covered zone, moving with a component
toward/away from the sensor. Perform a committed, realistic forward fall onto
the mattress. Stay down for ~5 s, then sit up.

**[DASHBOARD]** Cut the dashboard to full-frame at the moment of the fall:
status card flips to the red **FALL CONFIRMED** alert (source `radar_ml`; a
parallel rules event may also appear — that is expected), the dot in Room view
turns **red**, and the fall event enters Recent activity. (Banner reads
"FALL CONFIRMED" — the §0 label edit is already applied; verify the Pi has
pulled it.)

**[PHONE INSERT]** Close-up of the phone lock screen receiving the Telegram
notification: `🚨 FALL_CONFIRMED detected at HH:MM:SS`. This insert can be
captured in a separate take and edited in at this point.

**[ACTION cont.]** Reach the laptop (or have "the caregiver" do it) and press
**Acknowledge**; the dashboard returns to normal.

**[VO]**
> "And now, the event this system exists for. I fall. Within roughly half a
> second, the learned detector classifies the motion pattern — the downward
> velocity, the collapse in tracked height, the change in the point cloud —
> and raises a fall alert. The dashboard turns red, my track turns red on the
> floor plan, and, most importantly, the caregiver's phone receives a Telegram
> notification within seconds. In a replay of our one hundred twenty-three
> recorded sessions, this detector caught every fall with zero false alarms —
> a result I qualify in the write-up, because that replay uses the final model
> on its own recordings. Once the person is attended, the caregiver
> acknowledges the alert, and the system goes back to watching."

### Scene 5 — Immobility / possible faint (~90 s, includes a ~40 s wait)

**[ACTION]** Lower yourself to the floor (a controlled fall or simply lie
down on the mattress at floor level) and **stay completely still for at least
45–50 seconds**. Then get up. (The detector needs a 5 s stability window fully
filled *before* the 30 s immobility count starts, and any single frame of
movement resets it — so real time to alert is ~35–40 s, and 30 s exactly will
routinely fail.)

**[DASHBOARD]** If the initial movement fires a fall alert, either have a
second person ("the caregiver") acknowledge it at the laptop, or simply leave
it un-acknowledged — the immobility alert will replace it on the banner. **The
subject must not move to press anything**: reaching over resets the stillness
counter and the faint never fires. After ~35–40 s of floor-level immobility
the status flips to **IMMOBILITY ALERT** and a `Possible faint` event appears;
Telegram fires again.

**Contingency:** the camera operator watches Room view during the wait — a
motionless person is the hardest radar target, and if the track drops, all
counters restart from zero on re-acquisition. If the dot disappears, or no
faint has fired after ~60 s of stillness, abort (wiggle a hand, get up, clear
history) and retake.

**[VO]** (the 30 s wait is covered by narration; time-lapse the middle in the
edit if needed)
> "Falls are not the only emergency. The second dangerous pattern is a person
> on the floor who stops moving — a faint, a collapse, a fall that happened
> when nobody was watching. The system tracks the height of my reflection
> cluster: I am at floor level, and I am not moving. It waits thirty seconds —
> long enough to rule out picking something up — and then raises a separate
> immobility alert. This one is rule-based and deliberately conservative: at
> floor level, motionless, for half a minute, there is no innocent
> explanation. Again — the dashboard, and the caregiver's phone."

### Scene 6 — OPTIONAL bonus: the lateral fall (~45 s)

Record only if scenes 1–5 are in the can and energy allows.

**[ACTION]** Move the mattress to the lateral side of the room (motion mostly
across the radar beam). Fall onto it.

**[DASHBOARD]** ML fall alert fires. (If it doesn't on the first try, this
scene is cut — no harm. If you retry, wait ~30 s between attempts: the rules
detector has a 25 s per-track cooldown; the ML cooldown is only 5 s, but
waiting keeps both events available.)

**[VO]**
> "One more, and this one is harder than it looks. Radar measures velocity
> along the beam, so a fall moving across it is nearly invisible to
> Doppler — our rule baseline missed four of thirteen falls on this side of
> the room. The learned model, using height and shape features rather than
> velocity alone, caught all of them in replay. Here it is, live."

### Scene 7 — Close (~20 s)

**[ACTION]** Stand by the laptop, scroll the Recent activity list showing the
session's events.

**[DASHBOARD]** Full event history of the demo: presence, fall, faint, all
timestamped. Radar still online.

**[VO]**
> "Presence, tracking, everyday activity with no false alarms, a detected
> fall, and an immobility alert — all from a single radar sensor, processed
> locally, with no camera and no cloud. That is SafeRoom."

---

## 4. Video 2 — Explanation script (standalone, ~5 min)

Format: Guillermo speaking to camera, optionally cutting to stills/clips of
the demo and the dashboard. Written to be read nearly verbatim at 140–150 wpm
(~700 words ≈ 5 min). Section breaks are natural cut points.

### 4.1 What you just saw (~40 s)

> "In the demo you saw a live fall-detection system running in a real bedroom.
> Nothing was staged after the fact: the dashboard, the alerts, and the
> Telegram notifications were all produced in real time by the system while I
> was in the room. In the next few minutes I want to explain what was actually
> happening under the hood — what the sensor sees, how the detection works,
> and what the system can and cannot do."

### 4.2 The setup (~50 s)

> "The room is five point one by three point seven meters. On the short wall,
> at two point zero four meters high and tilted ten degrees down, there is a
> Texas Instruments IWR6843 millimeter-wave radar — a single small board with
> the antennas built in. Next to it, a Raspberry Pi 5. The radar transmits
> chirps at sixty gigahertz and measures range, radial velocity, and angle for
> every reflection, twenty times per second. Its firmware — which I modified
> for this project — clusters those reflections, tracks each person with a
> Kalman filter, and streams a compact binary summary to the Pi over a serial
> link. Everything you saw on the dashboard was computed on that Pi."

### 4.3 What the dashboard shows (~50 s)

> "The dashboard is worth a second look, because what it does *not* show is
> the point. The floor plan shows one dot per tracked person — position and
> track ID, nothing else. That dot is derived from a sparse cloud of radar
> points; there is no image behind it to leak. The thermal panel is a
> thirty-two by twenty-four pixel infrared array — seven hundred sixty-eight
> pixels for the whole room — where a person appears as a blob a few pixels
> wide. Neither modality can identify a face or record what you look like.
> The activity list and the alert banner are driven by events: presence,
> fall, immobility. That event stream, not video, is what a caregiver sees."

### 4.4 How detection works (~80 s)

> "Detection runs in two layers, in parallel. The first is a rule baseline:
> if the tracked vertical velocity exceeds a threshold for several consecutive
> frames, that is a fall; if the person's reflection sits at floor level,
> motionless, for thirty seconds, that is an immobility alert. Rules are
> transparent, but brittle — in our evaluation they raised roughly ten false
> alarms per hour and missed falls moving across the radar beam, where
> Doppler velocity nearly vanishes.
>
> The second layer is the learned detector, and it is the heart of the
> project. Every frame, a sliding window of the last one and a half seconds
> is summarized into thirty-six features — height statistics, vertical
> velocity and acceleration, the shape and spread of the point cloud, and
> their temporal trends — and an XGBoost classifier scores the window. I
> trained it on a labeled dataset of one hundred twenty-three sessions I
> recorded in this room: twenty-eight real falls plus deliberate confounders
> like sitting down, crouching, and lying on the bed. Under
> leave-one-session-out cross-validation it reaches a recall of zero point
> seven six at a precision of zero point six three at the window level; at
> the event level, replaying every session, it caught all twenty-eight falls
> with zero false alarms — although I should be precise: that replay uses the
> final deployed model, so it demonstrates latency and integration rather
> than out-of-sample generalization."

### 4.5 Honest limits (~45 s)

> "Three limits, stated plainly. First, the dataset has a single subject —
> me — so the validation is session-level, not subject-level; recording more
> people is the first item of future work. Second, the thermal camera you saw
> streaming was evaluated as a confirmation sensor for radar alarms, and that
> fusion is reported in the thesis as a negative result — it vetoed real falls
> — so in this demo it contributes visualization, not decisions. Third, the
> lateral blind spot is physics: Doppler cannot see motion across the beam.
> The learned model compensates for it much better than the rules, but the
> definitive fix is a second sensor."

### 4.6 Close (~20 s)

> "A single radar, a Raspberry Pi, no video camera, no cloud, no wearable —
> and a system that detects presence, falls, and immobility in real time, and
> puts an alert on a caregiver's phone within seconds. Thank you for
> watching."

---

## 5. Editing notes

- **Sync:** align each take by the clap (audio spike in both recordings).
- **Layout:** room footage full-frame; dashboard PiP at ~35–40% width in a
  corner; cut dashboard to full-frame for 2–3 s at every alert flip; Telegram
  phone insert full-frame for ~3 s after the fall alert.
- **Scene 5 wait:** show the first ~8 s and the last ~8 s of the ~35–40 s
  immobility wait; bridge with a timestamp overlay or a 4× time-lapse —
  keep the dashboard clock visible so the timing stays verifiable.
- **VO recording:** record after picture lock, scene by scene, quiet room,
  phone or USB mic ~20 cm away. Read at a calm pace; the scripts above are
  sized to their scenes.
- **Do not show** the Telegram bot token, the Tailscale IP, or any terminal
  with credentials. If a terminal appears in frame, keep only the two launch
  commands visible.
- **Music:** optional, low, none during alert moments (the silence sells it).

---

## 6. Fact table (verified against the codebase)

Everything the VO claims, with its source — keep numbers consistent if you
re-edit the narration:

| Claim in VO | Source |
|---|---|
| Room 5.10 × 3.70 m; sensor at 2.04 m, 10° tilt | `SafeRoom_1p9m_4x6m.cfg`, CLAUDE.md |
| 20 fps frame rate | `frameCfg` 50 ms |
| Rules: vz ≤ −1.15 m/s for 3 frames (Tier 1) | `radar_reader.py` FallDetector |
| Immobility: floor level (maxZ < 0.80 m), stable, 30 s (Tier 2) | `radar_reader.py` FallDetector |
| ML: 30-frame (1.5 s) window, 36 features, XGBoost | `feature_engineering.py`, `ml_inference.py` |
| Dataset: 123 sessions, 28 fall events, single subject | thesis report / CLAUDE.md |
| LOSO window-level: recall 0.76, precision 0.63 | thesis results |
| Replay: 28/28 falls, 0 false alarms, median latency 0.3 s (in-sample) | thesis results |
| Rules baseline: ~10.5 false alarms/hour, missed 4/13 lateral falls | thesis results |
| Thermal fusion = rejected negative result; demo runs `--ir` headless (no fusion) | thesis results, runbook 09 |
| Telegram fires only for `fall_confirmed`, `fall_fast`, `fall_failopen`, `faint` | `dashboard_server.py` `TG_CRITICAL_TYPES` |
| ML event = `fall_confirmed`, source `radar_ml` | runbook 09 §3 |

> **Before recording:** the rows sourced "thesis results" (10.5 FA/h, 0.63
> precision, 4/13 lateral, 0.3 s median latency, 28/28) live only in the
> report, not in the repo — cross-check each against the final thesis tables
> (note CLAUDE.md says "~28 fall events"; confirm the exact count) and pin the
> Source column to the specific table/figure. Keep VO latency wording ("within
> roughly half a second") consistent with whichever number the thesis reports.

---

## 7. Review changelog

Reviewed by the ml-engineer agent on 2026-07-23 (15 findings, all applied):
scene-5 timing corrected to the real ~35–40 s trigger (5 s stability window
must fill before the 600-frame count starts; ≥45–50 s of stillness scripted);
in-sample caveats added to every replay claim in scenes 3/4/6; "no camera" →
"no video camera"; ack procedure in scene 5 fixed so the subject never moves;
ML false-positive and track-loss contingencies added; track-ID warm-up rule
(≥2 s stable TID before falling) added; mattress-height maxZ < 0.80 m check,
Telegram day-of dry-run, laptop sleep/notification lockdown, and pre-clap
dashboard-liveness check added to pre-flight; Telegram latency wording relaxed
to "within seconds"; rules 25 s cooldown noted for scene-6 retries.
