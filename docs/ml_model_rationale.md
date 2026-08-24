# Why a learned model, given a working rule-based detector

This note answers the obvious question: if `FallDetector`
(`saferoom.radar.detector`) already detects falls, what does training a
classifier add?

## The arguments, strongest first

### 1. The rules are brittle and do not generalize

`FallDetector` is a pair of hand-calibrated fixed thresholds:

- Tier 1 FAST: `vz <= -1.15 m/s` for 3 consecutive frames.
- Tier 2 FAINT: `maxZ < 0.80 m` and `std(z) < 0.10 m` for 30 s.

Those numbers hold in one specific room, at the sensor height (2.04 m) and tilt
(10 deg) used during calibration. Change the scenario, whether a different room,
a different mounting height, a different person, or an atypical fall, and the
thresholds stop being optimal. A model trained on multivariate features learns
the *statistical structure* of a fall rather than one threshold.

### 2. Rules look at one feature at a time; the model looks at 36 jointly

The rule-based detector evaluates `vz`, then `maxZ`, then `std(z)`,
sequentially and independently. XGBoost evaluates all **36 features** produced
by `saferoom.ml.features` (Kalman accelerations, point-cloud statistics,
temporal variances) **jointly**, and finds non-linear interactions nobody would
hand-code. For example: moderate `vz` *but* high cluster dispersion *and* rising
Z acceleration.

### 3. The precision/recall trade-off becomes measurable

With hand-tuned rules there is no rigorous way to quantify the trade-off, only
"it seems to work". `saferoom.ml.evaluate` produces confusion matrices, ROC and
PR curves, and latency distributions over LOSO cross-validation. That is what
the work has to defend: reproducible metrics rather than anecdote. The rules are
the *baseline*; the model is what gets compared against it.

### 4. Classes the rules cannot separate

The rule-based approach has a documented intrinsic limit: Tier-2 SLOW could not
distinguish a slow collapse from voluntarily sitting on the floor, and was
removed for that reason (see [limitations.md](limitations.md)). A classifier
trained on labelled examples of `fall / near_fall / sit / lie / walk / stand /
none` can learn that boundary, provided the data contains it.

### 5. Extensibility

Adding a class to the rule-based detector means rewriting logic and
re-calibrating thresholds by hand. Adding it to the model means labelling data
and retraining. The second scales; the first does not.

## The framing

The heuristic detector is the system's baseline and shows that the radar signal
alone is sufficient to detect falls under controlled conditions. The XGBoost
model is the methodological contribution: it turns a manually calibrated system
into one **learned from data**, which makes it possible to (a) quantify
performance with standard metrics, (b) generalize beyond the calibration set,
and (c) extend to new activity classes without rewriting logic.

## Anticipated objections

**"The rule-based detector already works."**
It is the baseline. Without it there is nothing to compare the model against.
The model does not replace it: with `--ml-model`, both run in parallel in
`saferoom-read`.

**"Isn't XGBoost overkill here?"**
The complexity is justified by the dimensionality of the feature space (36) and
by the need to quantify performance under LOSO cross-validation.

**"Why not a neural network?"**
On a small dataset from a single subject and a single hardware setup, gradient
boosting is the right choice: strong performance from few samples, interpretable
through feature importance, and deployable on a Raspberry Pi with no GPU.
LSTM and 1D-CNN variants are implemented as a future option for when the dataset
grows.

## Dataset limitation and future work

### Session-level LOSO does not prove subject independence

The labelled dataset (123 sessions, ~28 fall events, ~173 fall windows, heavily
class-imbalanced) was recorded with a **single subject**. Cross-validation is
Leave-One-**Session**-Out, not leave-one-subject-out. With one person, every fold
trains and evaluates on the same individual's radar signature, so:

- subject independence cannot be demonstrated. The model may be learning one
  individual's body, gait and fall dynamics rather than the statistical
  structure of a fall in general;
- the reported metrics (pooled recall 0.757, zero false alarms in replay) should
  be read as **potentially optimistic** with respect to cross-person
  generalization. They are valid within the calibration conditions, but they do
  not demonstrate transfer to unseen people.

This is stated plainly because it is the first thing a reviewer will ask:
session-level LOSO is not subject-level LOSO.

### Closing the gap

The concrete route is to **record one or two additional subjects**, preferably
with different builds and movement patterns, following the same capture and
labelling protocol. With two or more subjects, **subject-level LOSO** becomes
possible: hold out an entire person per fold, and validate cross-person
generalization directly. Only then can the metrics be read as an estimate of
performance on unseen people rather than unseen sessions of the same person.

## Where this lives in the code

| Concern | Module |
|---------|--------|
| Rule-based baseline | `saferoom.radar.detector.FallDetector` |
| Extended logging | `saferoom.radar.ml_log` |
| Feature extraction | `saferoom.ml.features` |
| Training (LOSO-CV) | `saferoom.ml.train` |
| Evaluation and figures | `saferoom.ml.evaluate` |
| Deployment inference | `saferoom.ml.inference` |
