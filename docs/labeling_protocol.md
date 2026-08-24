# Labelling protocol

Version 1.1 (2026-07-08). Companion to `saferoom-label-mm`
(`saferoom.dataset.labeling_multimodal`).

Changes in v1.1, decided before dataset collection during the 2026-07-08
methodology review:

- New label **`fall_lying`** (key `g`), separating the impact (`fall`) from the
  time spent on the floor afterwards. The model trains on `fall` only; the IR
  fusion metric uses `fall` together with `fall_lying`. Previously a single
  `fall` stretched over 5 s had to serve two incompatible definitions and
  collided with the `lie` class.
- Window rule: a window counts as positive only if it contains **at least 5
  frames (250 ms) of `fall`**. Windows with fewer `fall` frames, or with
  `fall_lying`, are excluded from training (implemented in
  `saferoom.ml.features`).
- Using `fall_detected` (the rule-based output) as a label is forbidden.
  `saferoom.ml.features` raises an error if the `label` column is missing.

## Clock and propagation

Every label is stored in **two places**, kept consistent:

1. `radar.csv`, a `label` column per row. This is what the ML pipeline reads.
2. `manifest.json`, a `labels` list of
   `(t_start_mono_ns, t_end_mono_ns, label, notes)`. This is the canonical
   source for later multimodal re-analysis.

The labeller's IR cursor finds the thermal frame nearest the CSV's `t_mono_ns`,
so **`t_mono_ns` is always the temporal ground truth**.

## Vocabulary

| Label | Key | Apply when | Do not apply when |
|---|---|---|---|
| `fall` | `f` | Involuntary fall with impact or collapse to the floor. From the first frame with a clear `vz < 0` until roughly 2 s after impact, once the person is motionless on the floor. | Sitting, lying down, or crouching quickly but under control. Time spent on the floor afterwards is `fall_lying`. |
| `fall_lying` | `g` | Person on the floor after a fall, from the end of `fall` until the first attempt to get up. Must cover at least 5 s after impact (the IR confirmer's window). | Voluntary lying down, which is `lie`. The impact itself, which is `fall`. |
| `near_fall` | `a` | Visible loss of balance (trip, stumble) that does not reach the floor. | The person ends up on the floor. That is `fall`. |
| `sit` | `s` | Sitting down and remaining seated, including the transition. | Standing still, which is `stand`. |
| `lie` | `l` | Voluntarily lying down: bed, sofa, deliberately on the floor. | On the floor after a fall. That segment is `fall` / `fall_lying`. |
| `walk` | `w` | Continuous gait: visible steps, net displacement. | Stationary with micro-movements, which is `stand`. |
| `stand` | `t` | Standing without displacement; minimal sway is acceptable. | Starting to walk. Cut there and label the displacement `walk`. |
| `none` | `n` | Empty room. No tracks, or only isolated ghost tracks. | A person enters. Cut there and label what they do. |

## Boundary criteria

**Start of `fall`** — the first frame where `vz` becomes clearly negative
(below -0.5 m/s), or where the person has lost vertical control.

**End of `fall`** — the person is motionless on the floor for at least 2 s after
impact. From there, if they stay down, **label the rest `fall_lying`** until
they get up. Getting up belongs to neither label.

**`near_fall` vs `walk`** — use `near_fall` for a clear stumble (at least one
frame with anomalous `vz` and `az`) followed by recovery. When the call is
genuinely ambiguous, prefer `walk`.

**Long `none` blocks** — labelling 60 s in one span is fine.

**Consistency with the IR confirmer** — the confirmer evaluates the 5 s after
the radar event. The union of `fall` and `fall_lying` must cover that whole
interval with the body on the floor, so **`fall_lying` must run to at least 5 s
past impact**. `saferoom.evaluation.replay` automatically merges a `fall` with
its adjacent `fall_lying` when building ground-truth events, so a late detection
(a Tier-2 faint during the time on the floor) counts as a hit rather than a
false alarm.

## Practice

1. **Check IR before deciding.** Radar can be ambiguous during fast
   transitions: a small cluster on the floor is either a fall or a ghost. The
   thermal view settles it.
2. **Keep `fall` tight and `fall_lying` generous.** The impact ends about 2 s
   after touching the floor; everything after that is `fall_lying`. Getting up
   is in neither.
3. **`unknown` is forbidden in the final dataset.** Any row still marked
   `unknown` after labelling must become `none` or be discarded before
   training.
4. **Do not relabel without documenting it.** If the criterion changes, record
   what changed and why in `manifest.notes`.
5. **Sanity-check after labelling.** Run `saferoom-features sessions/<id>/` and
   confirm the number of extracted `fall` windows is plausible, at minimum one
   per fall session.

## Special cases

**Multiple people** — the collection setup does not guarantee per-subject
segmentation. Label the whole session by the dominant activity; if two
activities overlap, discard the session for v1 of the dataset.

**Ghost tracks** — if the pipeline mistakes a ghost track for a person, note it
in `manifest.notes`. Never label a ghost as `fall`.

**Short sessions (under 10 s)** — valid for `fall` and `near_fall` as long as
they include at least 3 s before the event. For continuous classes (`walk`,
`stand`, `none`) prefer sessions of 20 s or more.

## Expected result per session

- `radar.csv` with `label` other than `unknown` on 100 % of rows that have an
  active track.
- `manifest.json` with at least one `LabelSpan` in `labels`.
- A `.bak` backup of the CSV, created automatically on first save.
