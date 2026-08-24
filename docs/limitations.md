# Known limitations

Documented deliberately: each of these is a property of the sensing approach or
of the dataset, not a defect to be patched.

## Lateral movement blind spot

Doppler measures radial velocity only. Motion perpendicular to the beam, such as
walking across the room rather than toward or away from the sensor, produces
near-zero Doppler and is harder to track. This is not fixable in configuration;
it would require reorienting the sensor physically.

## Height values are relative, not absolute

TLV 1012 reports `maxZ` and `minZ` of the radar reflection cluster. Their
difference is the vertical extent of that cluster, typically 0.3 to 0.5 m, not
the person's height. Fall detection therefore uses the value as a ratio and as a
floor-proximity signal, never as an absolute stature threshold.

## `MAX_RESOLVED_OBJECTS_PER_FRAME` is fixed at 750

Hardcoded in the mmWave SDK. Changing it means recompiling the SDK. The firmware
UART cap in `mss_main.c` works around it without touching the constant; see
[architecture.md](architecture.md#the-firmware-point-cloud-cap).

## Slow collapse is indistinguishable from sitting on the floor

Both produce the same observable signal: the person was standing, and is now at
floor level for 20 s or more. A dedicated Tier-2 SLOW rule was implemented and
then removed, because it could not separate the two.

For the target user, an elderly person living alone, voluntarily sitting on the
floor is not a realistic scenario, so the case is not handled. Distinguishing it
would need a second sensor, for example an IR camera for posture.

## IR fusion fails closed on one side of the room

The MLX90640 does not register a warm blob for a person on one side of the room
(physically left, the radar's wide +X side). The radar tracks that region
normally, but the IR confirmer's Rule-0 pre-window ghost veto treats an empty
pre-window as evidence of a ghost track and **vetoes** the event, downgrading a
real fall to `fall_candidate` (log-only).

This is coverage-dependent fail-closed behaviour, not a bug: fusion cuts false
alarms but can suppress real falls where IR has no view. It is reported as the
project's fail-closed analysis.

Scope of the impact is limited to the IR fusion metric. The radar-only ML
detector, which is the primary contribution, never consults IR and is unaffected.

When analysing fusion results, split by the actual `x` coordinate in
`radar.csv`, not by session name. Some session names have `izq`/`dcha` swapped,
confirmed against X positions, so the name is unreliable.

## Single-subject dataset: session-level LOSO cannot prove subject independence

The labelled dataset (123 sessions, ~28 fall events, ~173 fall windows, heavily
imbalanced) was recorded with a **single subject**. Cross-validation is
Leave-One-**Session**-Out, not leave-one-subject-out: every fold still trains and
tests on the same person's radar signature.

The model may therefore be fitting one individual's body, gait and fall dynamics
rather than subject-independent fall structure, which makes the reported metrics
(pooled recall 0.757, zero false alarms in replay) potentially optimistic as an
estimate of cross-person generalization.

Addressing it is documented future work: recording one or two additional
subjects would enable subject-level LOSO. See
[ml_model_rationale.md](ml_model_rationale.md).
