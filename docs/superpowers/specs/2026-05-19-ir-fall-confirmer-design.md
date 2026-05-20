# IR Fall Confirmer — Design

**Author**: Guillermo (with Claude)
**Date**: 2026-05-19
**Status**: Approved, ready for implementation plan

## Context

SafeRoom's primary fall detector is the IWR6843AOPEVM mmWave radar. The
MLX90640 thermal camera (24×32 px) currently only feeds the live dashboard
and is recorded synchronously for dataset purposes. This spec extends the
camera's role into the **decision path** to reduce false-positive alerts
without sacrificing the recall of real falls.

The hard constraint is sensor resolution: at the target mount (2 m height,
person ~2 m away) a human body occupies ~8×5 pixels in the rotated image.
That rules out detailed motion analysis but is more than enough to classify
a static posture ("upright vs lying on floor"). The design exploits this.

## Goals

1. **Reduce false-positive radar alerts** without losing real falls.
2. **Provide ground-truth confirmation during dataset recording**, usable by
   the labeling tool and by an IR-only baseline model for thesis comparison.
3. **Stay simple and interpretable** so the chapter on multimodal fusion has
   a small, defensible algorithmic story.
4. **Fail safely**: an IR failure must never silence a radar alert.

## Non-goals

- The IR does NOT autonomously trigger fall alerts (no IR-only detection in
  the decision path for v1).
- No detailed kinetic event detection from IR ("did the body fall?") in
  v1's decision path. The kinetic exploration script that decides whether
  to build it is part of this spec (Extensions, phase 1).
- No replacement of the rule-based or ML radar fall detectors.

## Decisions (from grill-me session)

| # | Decision | Rationale |
|---|----------|-----------|
| 1 | IR role: both offline ground truth + live confirmer | Maximises tesis output (baseline model + fusion chapter + working system) |
| 2 | Live function: passive "lying on floor" confirmation; kinetic detection deferred | Resolution sufficient for static posture, insufficient for fast motion at 2 m |
| 3 | Trigger: reactive — only runs when radar fires fall event | Keeps radar primary; cheap; clear narrative |
| 4 | Window: 2 s before + 5 s after radar fall timestamp | Pre-window vetoes ghost-track FPs; 5 s post is enough for the body to settle |
| 5 | Algorithm: rule-based with 4 features | Zero data needed to start; interpretable; LogReg upgrade planned post-dataset |
| 6 | Fusion: confidence downgrade with fail-open | Never lose a true positive; reduce alarm fatigue when IR vetoes |
| 7 | Calibration: 30 s auto-calibration at startup with empty-room safeguard | Adapts to ambient; no operator action; safeguard prevents calibration-with-person bug |
| 8 | A_kinetic: phased — phase 1 (exploration script + decision criterion) shipped now; phase 2 detector built only if exploration shows ROC AUC ≥ 0.7 on labeled data | Avoids designing thresholds without data while still committing to the evaluation path; failure mode is itself a defensible thesis result |

## Architecture overview

```
┌──────────────────────────────────────────────────────────────────┐
│ radar_reader.py (live mode)                                      │
│                                                                   │
│  FrameReader ──► FallDetector ──► fall event (with t_mono_ns)    │
│       │                                  │                        │
│       │                                  ▼                        │
│       │                          IrConfirmer.evaluate(t_mono)     │
│       ▼                                  │                        │
│  IR thread (Mlx90640Capture) ───► IrRingBuffer                    │
│  pushes (frame, t_mono_ns)                │                       │
│                                           ▼                       │
│                                   posture decision +              │
│                                   reason payload                  │
│                                           │                       │
│                                           ▼                       │
│                              POST to dashboard with:              │
│                               event_type = FALL_CONFIRMED         │
│                                            | FALL_CANDIDATE       │
│                                            | FALL_FAILOPEN        │
└──────────────────────────────────────────────────────────────────┘
```

`IrConfirmer` lives in the same process as `radar_reader` to keep latency
low and to make the system self-contained. The dashboard server stays a
presentation/notification layer and trusts the event type it receives.

## Components

### `tools/ir_confirmer.py` (new)

Public surface:

```python
class IrConfirmer:
    def __init__(self, capture: Mlx90640Capture,
                 buffer_seconds: float = 10.0,
                 pre_window_s: float = 2.0,
                 post_window_s: float = 5.0,
                 hyperparams: IrConfirmerParams = ...,
                 calibration_seconds: float = 30.0):
        ...

    def is_calibrated(self) -> bool: ...
    def is_available(self) -> bool: ...   # fail-open feed

    def evaluate(self, radar_t_mono_ns: int) -> ConfirmerResult:
        """Block up to post_window_s, then classify. Returns:
            ConfirmerResult(
                decision: Literal['confirmed','vetoed','failopen'],
                confidence: float,                     # 0..1
                features: dict[str, float],            # per-rule outputs
                frames_used: int,
                window_t_mono_ns: tuple[int, int],
            )
        """
```

Internals:

- **Ring buffer** of the last 10 s of IR frames. Default refresh rate
  stays `--ir-hz 16` (≈ 8 fps effective, 80 frames in the buffer). The
  rate is a hyperparameter; for the passive lying-down classifier 8 fps
  is ample (the body is static by construction). The exploratory study
  for A_kinetic (see Extensions section) will report whether 16 Hz
  effective (`--ir-hz 32`) is needed there.
- **Background model**, learned in the first 30 s of operation (mean and
  std per pixel). Refreshed once if `is_available` reports a drop (e.g.,
  after a 5 min outage).
- **Empty-room safeguard**: at end of calibration window, reject the
  calibration if there is a *clustered* warm region inconsistent with an
  empty room — defined as:
    `count(pixel > mean(frame) + 5 °C) >= 5` **and**
    `max(frame) - mean(frame) >= 8 °C`.
  Both conditions are required so that a single warm pixel of sensor
  noise does not abort calibration, while a person's face/hands (≥ 5 px
  cluster at ~8 °C over ambient) trips it reliably. Absolute thresholds
  are avoided because a hot summer day in Madrid can push ambient over
  30 °C without anyone in the room. Hyperparameters:
  `safeguard_cluster_size = 5`, `safeguard_cluster_dt = 5.0`,
  `safeguard_peak_dt = 8.0`. On failure, retry is scheduled every 60 s
  silently; status surfaces in the dashboard status bar.
- **Rule evaluator** runs only inside `evaluate()`. Sees the window
  `[radar_t_mono - 2 s, radar_t_mono + 5 s]`. If `pre` part is missing
  (system just started), uses post only and marks `pre_available=False`.

### Rule-based features (v1)

Notation: `bg(y,x)` = calibrated mean background; `frame[y,x]` =
post-rotation temperature (rotation comes from the existing `--ir-rotate`
CLI flag); `H,W` = post-rotation dimensions (32 high × 24 wide if
`--ir-rotate 90`). The blob "centroid" is the unweighted mean of the
`(y,x)` coordinates of pixels in `mask` (computed per frame).

**Rule 0 — Pre-window ghost-track veto** (only when the 2 s pre-window is
available): if `mask.sum() < MIN_PIXELS` in ≥ 80% of pre-frames, there
was no warm body in the room before the radar event. Decision is
immediately **VETOED** without evaluating rules 1–4 (typical ghost track
from static reflections).

1. **Hot blob presence** (post window only):
   `mask = frame > bg + 3·std` per frame.
   Feature `hot_blob_fraction` = fraction of post-window frames with
   `mask.sum() >= MIN_PIXELS` (default 10). Threshold ≥ 0.6.

2. **Floor zone occupancy** (post window only):
   `floor_zone` = bottom 1/2 of frame (configurable). Feature
   `floor_centroid_fraction` = fraction of post-window frames whose blob
   centroid `y` is inside the floor zone. Threshold ≥ 0.7.

3. **Horizontal aspect ratio** (post window only):
   Per frame compute the bounding box of `mask`. `aspect = bbox_height /
   bbox_width`. Feature `horizontal_fraction` = fraction of post-window
   frames with `aspect < 0.7`. Threshold ≥ 0.5.

4. **Stillness** (last 3 s of post window):
   `centroid_std_px` = std of blob centroid position across the last 3 s.
   Threshold ≤ 4 px.

The decision is **CONFIRMED** if 3 of the 4 features pass their threshold,
otherwise **VETOED**. The 3-of-4 rule absorbs single-feature noise
(e.g., a person sitting motionless will fail "horizontal" but pass the
other 3, still confirming — which is OK because radar already gated it as
a fall candidate). Confidence is exposed as `passed / 4`.

All thresholds and the `floor_zone` ratio are hyperparameters
(`IrConfirmerParams` dataclass) and tuned during the post-installation
calibration session, not hard-coded.

### Fusion policy (in `radar_reader._read_loop` or new helper)

```python
if fall_detector.is_fall(...):
    if not ir_confirmer.is_available():
        emit('FALL_FAILOPEN', radar_features=..., ir_reason='unavailable')
    else:
        result = ir_confirmer.evaluate(t_mono)
        if result.decision == 'confirmed':
            emit('FALL_CONFIRMED', radar_features=..., ir=result)
        else:
            emit('FALL_CANDIDATE', radar_features=..., ir=result)
```

Dashboard semantics:
- `FALL_CONFIRMED` → notify carer + Telegram + dashboard banner.
- `FALL_FAILOPEN`  → same as CONFIRMED for alerting; tagged so the
  tesis evaluation can split them.
- `FALL_CANDIDATE` → log only, visible in dashboard timeline but no
  notification.

Faint detection (tier-2) is **unchanged** by this spec. The IR confirmer
only acts on tier-1 (kinetic) falls.

### Calibration UX

`radar_reader.py` and `session_recorder.py` print clear console messages:

```
[IR] calibrating 30 s — keep the room empty ...
[IR] calibration OK — mean ambient 22.4 °C, max bg pixel 24.1 °C
```

or, on failure:

```
[IR] calibration aborted — hot pixels detected (max 34.7 °C).
[IR] Will retry in 60 s. Person in view?
```

### Logging for tesis evaluation

Every fall event (regardless of fusion outcome) is appended to
`logs/fall_events.jsonl`:

```json
{"t_mono_ns": ..., "t_wall": ..., "event_type": "FALL_CONFIRMED",
 "radar": {"tid": 1, "vz_peak": -1.52, "tier": "fast"},
 "ir": {"decision": "confirmed", "confidence": 0.75,
        "features": {"hot_blob_fraction": 0.94, ...},
        "frames_used": 56, "pre_available": true}}
```

This file is the basis for the fusion comparison in the tesis:
precision/recall of radar-only vs. radar+IR vs. radar+IR(fail-open).

## Extensions (phased build)

### A_kinetic (build the decision criterion now; the detector itself is conditional)

**Hypothesis**: even at 24×32 with 8 fps effective, a fall produces a
detectable kinetic signature (centroid vertical velocity, blob bounding
box vertical extent collapse). If true, IR can detect falls
independently of radar — and the project gains a computer-vision angle.

**Phase 1 — build now (alongside the confirmer)**:
- `tools/explore_ir_kinetics.py`: reads labeled session dirs (radar.csv +
  thermal.npz + manifest.labels), extracts the candidate kinetic features
  below per labeled event window, and produces per-class boxplots + ROC
  curves into `figures/ir_kinetics/`. The script is data-ready
  immediately (only depends on session schema, which is already shipped).
- Candidate features (initial set, refinable):
  - centroid vertical velocity (px/s), peak magnitude over 250 ms
  - bbox height delta over 250 ms windows
  - bbox aspect ratio rate of change (px/s)
  - pixel-wise temporal gradient magnitude, frame-level peak

**Phase 2 — run after first labeled hardware sessions**:
Execute `explore_ir_kinetics.py` on the data. Decision rule:
- If at least one feature shows ROC AUC ≥ 0.7 on a held-out session →
  draft a separate spec `2026-XX-XX-ir-kinetic-detector-design.md`,
  go to phase 3.
- Else → document the negative result in the tesis ("MLX90640 at 2 m
  cannot independently detect falls; passive confirmation is the only
  viable IR contribution") and close the question.

**Phase 3 (conditional)**: implement `IrKineticDetector` as an
*additional* signal alongside the radar primary; fusion remains as
described in the main spec but with a third input.

### LogReg upgrade for the passive classifier (post-dataset)

After ≥ 30 labeled fall sessions are collected, train a logistic
regression on the four current features (plus optionally `centroid_y_mean`,
`bbox_height_mean`, `bbox_width_mean`) using LOSO-CV. Replace the 3-of-4
voting if F1 improvement is statistically significant on held-out
sessions.

### LogReg upgrade for B (post-dataset)

After ≥ 30 labeled fall sessions are collected, train a logistic
regression on the four current features (plus optionally `centroid_y_mean`,
`bbox_height_mean`, `bbox_width_mean`) using LOSO-CV. Replace the 3-of-4
voting if F1 improvement is statistically significant on held-out
sessions.

## Files touched

| Path | Action | Purpose |
|------|--------|---------|
| `tools/ir_confirmer.py` | **new** | `IrConfirmer`, `IrConfirmerParams`, `ConfirmerResult`, ring buffer, background model |
| `tools/explore_ir_kinetics.py` | **new** | A_kinetic phase 1 — features + ROC plots from labeled sessions |
| `tools/radar_reader.py` | modify | Wire up confirmer when `--ir` is active; emit new event types; expose `--no-confirmer` for radar-only evaluation |
| `tools/saferoom_notifier.py` | modify | Route by event_type (CONFIRMED/FAILOPEN → notify, CANDIDATE → silent) |
| `tools/dashboard_server.py` | modify | Accept and persist new event_type field; tag banner colors |
| `tools/dashboard/index.html` | modify | Render the three event types distinctly |
| `tools/feature_engineering.py` | (later) | Add IR features when LogReg upgrade is taken |
| `docs/data_collection_protocol.md` | modify | Note that recording sessions should include 30 s empty-room at start |
| `docs/labeling_protocol.md` | modify | Note that `fall` label scope can span post-fall lying period (no change to vocab) |

## Resolved questions

1. **Dashboard visibility of the three event types**: during development
   the dashboard shows them distinctly (CONFIRMED = red banner + Telegram,
   FAILOPEN = red banner with subtle "(IR offline)" badge + Telegram,
   CANDIDATE = orange row in the event timeline, no Telegram). This makes
   the fusion behaviour visible while debugging. The "final
   demo" trim (collapse FAILOPEN visuals into CONFIRMED, hide CANDIDATE)
   is left as a config knob and will be flipped before the tesis defence
   if Guillermo prefers a cleaner look at that point.
2. **Calibration retry when room is occupied**: silent retry every 60 s.
   Calibration status is exposed in the dashboard status bar
   (`IR: calibrating…` / `IR: ready` / `IR: room not empty, retrying`).
3. **`--no-confirmer` CLI flag**: added to `radar_reader.py`. Disables the
   IR confirmer entirely; emits only legacy `FALL` events. Required for
   the tesis evaluation that compares "radar-only" vs "radar+IR" head to
   head on the same hardware sessions.
