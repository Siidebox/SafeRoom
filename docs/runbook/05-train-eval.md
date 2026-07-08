# Entrenar y evaluar modelos

Asume que tienes >= 2 sesiones etiquetadas en `sessions/`. Para datos reales
serias >= 30 sesiones de caída + variación.

## Entrenar (XGBoost + Random Forest, opcional LSTM)

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/train_model.py --data sessions/*/ --no-dl
```

- `--data` acepta directorios de sesión (`sessions/<id>/`) o CSVs sueltos,
  con globs.
- `--no-dl` salta el LSTM (más rápido y sin necesidad de torch).
- Quita `--no-dl` si quieres el LSTM también.

Salida:
- `models/fall_detector_xgb.pkl`
- `models/fall_detector_rf.pkl`
- `models/cv_results.csv` (métricas LOSO-CV)

## Evaluar y generar figuras

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/evaluate_model.py --data sessions/*/ --no-dl --outdir figures/eval --latex
```

Salida en `figures/eval/`:
- `confusion_matrices.png`
- `roc_curves.png`
- `pr_curves.png`
- `feature_importance.png`
- Tabla LaTeX por stdout (con `--latex`).

## A_kinetic — fase 1 (¿la IR detecta caídas por sí sola?)

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/explore_ir_kinetics.py sessions/ --out figures/ir_kinetics
```

Para cada label-span calcula 4 features cinéticas IR
(`centroid_vy_peak`, `bbox_height_drop`, `aspect_change_peak`,
`temporal_grad_peak`) y dibuja sus curvas ROC.

Decisión (criterio del spec):

- Si **alguna** feature alcanza ROC AUC >= 0.7 → escribir spec separado
  para `IrKineticDetector` (fase 2).
- Si no → A_kinetic se descarta con justificación cuantitativa.

## Comparar reglas vs ML vs fusión (para el TFM) — replay offline

La comparación se hace **offline sobre las mismas sesiones grabadas** (no con
runs live separados, que no serían un experimento pareado). El replay pasa
`radar.csv` + `thermal.npz` por `FallDetector`, `MlFallDetector` e
`IrConfirmer` con reloj de sesión y agrega las detecciones contra los
LabelSpans del `manifest.json`:

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/replay_session.py \
    sessions/*/ --ml-model models/fall_detector_xgb.pkl --json figures/replay_results.json
```

Salida por sesión y detector (`rules`, `ml`, `rules+ir`, `ml+ir`):

- **recall por evento** — fracción de caídas GT con >= 1 detección
- **FA/h** — falsas alarmas por hora de sesión
- **lat_med** — latencia mediana de detección desde el inicio del evento
- decisiones del IR confirmer por detección (`confirmed`/`vetoed`/`failopen`)

Flags útiles: `--ml-threshold` (umbral del modelo), `--tol-pre/--tol-post`
(tolerancias de matching detección↔evento en segundos).

Nota: en sesiones donde la habitación no está vacía los primeros 30 s, el
confirmer no calibra y reporta `failopen` (equivale a radar-only).
