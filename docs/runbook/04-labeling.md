# Labelling recorded sessions

Labelling is post-hoc: record first, label afterwards. The criteria for each
label live in [labeling_protocol.md](../labeling_protocol.md); this page is how
to drive the tool.

## What gets labelled

| Labelled | Sessions |
|---|---|
| Yes | `fall`, `nearfall`, `sit`, `walk`, `lie`, `stand`, `none`, negative, continuous-scenario |
| No | `calib_*`, which is thermal background for the IR confirmer, not data |

Start with the `fall` sessions. They matter most and they are the only ones with
two spans.

## Multimodal labeller (recommended)

```bash
saferoom-label-mm sessions/<id>/
```

One session at a time. It needs a Qt window, so run it from a desktop terminal,
not over SSH. The window shows:

- top: `maxZ` (height) over time
- middle: `vz` (vertical velocity) and `az`
- bottom left: the MLX90640 heatmap for the frame nearest the cursor
- bottom right: a colour bar for the current label

### Controls

| Action | How |
|---|---|
| Select a time range | **Drag** across the top or middle panel |
| Assign a label to the range | Press the label's key |
| Scrub the IR heatmap through time | Move the mouse over the plots |
| Undo the last assignment | `z` |
| Save and exit | `Enter` — writes the `label` column in `radar.csv` **and** the `LabelSpan` list in `manifest.json` |
| Exit without saving | `Esc` |

### Label keys

| Key | Label | When |
|---|---|---|
| `f` | `fall` | From the first frame where `vz` is clearly negative (below -0.5 m/s) to about 2 s after impact |
| `g` | `fall_lying` | From the end of `fall` until they start getting up. Must cover **at least 5 s** after impact, the IR confirmer's window |
| `a` | `near_fall` | A stumble that never reaches the floor |
| `s` | `sit` | The transition plus remaining seated |
| `l` | `lie` | Voluntarily lying down, as opposed to being on the floor after a fall |
| `w` | `walk` | Gait with net displacement |
| `t` | `stand` | Standing without displacement |
| `n` | `none` | Empty room, nobody present |

The first time you save a session the tool writes a `radar.csv.bak` backup, so
relabelling is safe. `unknown` is forbidden in the final dataset: every relevant
frame must end up with a label.

## Worked example: one fall session

The recorded script was: stand 5 s, fall, stay motionless on the floor for at
least 8 s, get up. That needs **two spans**, `f` then `g`.

1. **Find the fall.** The `vz` panel shows a pronounced trough, typically
   `vz <= -1.15 m/s`. That downward spike is the moment of the fall. In the
   `maxZ` panel, height drops from standing (about 1.2 to 1.7 relative) to floor
   level (about 0.3 to 0.5).

2. **Mark `fall`.** Drag from the first frame where `vz` starts dropping clearly
   to about 2 s after impact, once height has settled at floor level and `vz` is
   back near zero. Press `f`. The span turns red.

3. **Mark `fall_lying`.** Drag from the end of that span to just before they
   start getting up, where `maxZ` begins rising again. Make sure it lasts at
   least 5 s. Press `g`. The span turns dark red.

```
 vz │      standing          on the floor        getting up
    │ ───────────────╮                          ╭──────────
    │                 ╲   trough = the fall     ╱
    │                  ╰─────╮          ╭──────╯
    │                        ╰──────────╯
    └──────────────────────────────────────────────────────► t
                    [====f====][========g========]
                      fall        fall_lying (>= 5 s)
```

4. **Review and save.** `z` undoes the last span. When both look right, press
   `Enter`.

## The other activities

| Session | Spans to mark |
|---|---|
| `nearfall_*` | One `a` span over the stumble. No floor phase, so no `g` |
| `sit_*` | One `s` span covering the descent and remaining seated |
| `lie_*` | One `l` span covering lying down and staying down |
| `walk_*` | One `w` span over the whole walk |
| `stand_*` | One `t` span over the standing period |
| `none_*` | One `n` span over the whole session |
| negative | **Several** spans, one per segment, labelled by what was happening (walk, stand, sit, none). No `fall` may appear |
| continuous | **Several** spans following the sequence: `w` while walking, `t` or `s` when stopping or sitting, and `f` + `g` at the fall exactly as above |

## Radar-only fallback

If a session has no `thermal.npz`, or you only want the radar view:

```bash
saferoom-label sessions/<id>/radar.csv
```

Same keys as the legacy tool: `f`, `n`, `s`, `w`.

## Sanity check after labelling

```bash
saferoom-features sessions/<id>/
```

It prints how many windows were extracted and how many are `fall` versus
normal. A fall session that yields no `fall` window means the labelling needs
another look, usually because the `f` span is shorter than the 5-frame minimum.
