# Training and evaluating models

Assumes at least two labelled sessions in `sessions/`. For results worth
reporting you want 30 or more fall sessions plus the required variation; see
[data_collection_protocol.md](../data_collection_protocol.md).

## Train

```bash
saferoom-train --data sessions/*/ --no-dl
```

- `--data` accepts session directories or loose CSVs, and supports globs.
- `--no-dl` skips the LSTM, which is faster and does not need PyTorch. Drop the
  flag to train it too.

Outputs:

- `models/fall_detector_xgb.pkl`
- `models/fall_detector_rf.pkl`
- `models/cv_results.csv` — LOSO-CV metrics

Cross-validation is **Leave-One-Session-Out**. With a single-subject dataset
that is not the same as leave-one-subject-out; see
[limitations.md](../limitations.md).

## Evaluate and generate figures

```bash
saferoom-evaluate --data sessions/*/ --no-dl --outdir figures/eval --latex
```

Writes to `figures/eval/`:

- `confusion_matrices.png`
- `roc_curves.png`
- `pr_curves.png`
- `feature_importance.png`

`--latex` additionally prints a metrics table to stdout.

## Compare rules vs ML vs fusion (offline replay)

The comparison runs **offline over the same recorded sessions**, not as separate
live runs, which would not be a paired experiment. Replay pushes `radar.csv`
and `thermal.npz` through `FallDetector`, `MlFallDetector` and `IrConfirmer`
with an injected session clock, then aggregates the detections against the
`LabelSpan` entries in `manifest.json`:

```bash
saferoom-replay sessions/*/ \
  --ml-model models/fall_detector_xgb.pkl \
  --json figures/replay_results.json
```

Reported per session and per detector (`rules`, `ml`, `rules+ir`, `ml+ir`):

- **event recall** — fraction of ground-truth falls with at least one detection
- **FA/h** — false alarms per hour of session
- **lat_med** — median detection latency from the start of the event
- the IR confirmer's decision per detection: `confirmed`, `vetoed`, `failopen`

Useful flags: `--ml-threshold` sets the model's decision threshold;
`--tol-pre` / `--tol-post` set the matching tolerance between a detection and an
event, in seconds.

Note: in sessions where the room was not empty for the first 30 s the confirmer
never calibrates and reports `failopen`, which is equivalent to radar-only.

Use `--ir-calib sessions/<calib_id>` to supply a dedicated empty-room background
instead, which is what the capture protocol records at the start of each block.
