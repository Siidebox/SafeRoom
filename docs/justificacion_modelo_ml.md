# Justificación del uso de XGBoost frente al detector por reglas

Notas para la defensa del TFM / redacción de la memoria. Responde a la pregunta:
*"¿Por qué entrenar un modelo si el detector por reglas (`FallDetector` en `tools/radar_reader.py`) ya detecta caídas?"*

## Argumentos, de más fuerte a más débil

### 1. El detector por reglas es frágil y no generaliza
El `FallDetector` actual son umbrales fijos calibrados a mano:
- Tier 1 FAST: `vz ≤ −1.15 m/s` durante 3 frames consecutivos.
- Tier 2 FAINT: `maxZ < 0.80 m` y `std(z) < 0.10 m` durante 30 s.

Esos números funcionan en la habitación concreta, con la altura de sensor (2.04 m)
y tilt (10°) usados durante calibración (2026-04-20). Cambia el escenario —
otra habitación, otra altura, otra persona, una caída atípica — y los umbrales
dejan de ser óptimos. Un modelo entrenado sobre features multivariadas aprende
la *estructura estadística* de una caída, no un umbral concreto.

### 2. Reglas = 1 feature a la vez; ML = 36 features conjuntas
El detector por reglas evalúa `vz`, luego `maxZ`, luego `std(z)`, de forma
secuencial e independiente. XGBoost evalúa las **36 features** producidas por
`feature_engineering.py` (aceleraciones Kalman, estadísticas del point cloud,
varianzas temporales, etc.) **conjuntamente** y descubre interacciones no
lineales que un humano no codificaría a mano. Ejemplo: "vz moderado **pero**
con dispersión alta del cluster **y** aceleración Z creciente".

### 3. Falsos positivos / falsos negativos medibles
Con el detector por reglas no se puede cuantificar rigurosamente el trade-off
precision/recall — sólo "parece que funciona". `evaluate_model.py` produce
matrices de confusión, ROC, PR y latencia sobre LOSO-CV. Eso es lo que un TFM
necesita defender: **métricas reproducibles**, no anécdotas. El detector por
reglas es el *baseline*; el ML es lo que se compara contra él.

### 4. Distinguir clases que las reglas no distinguen
La propia `CLAUDE.md` documenta la limitación:
> Tier-2 SLOW cannot distinguish slow collapse from voluntary floor-sitting.

Es una limitación intrínseca del enfoque por reglas. Un clasificador entrenado
con ejemplos etiquetados de `fall / near_fall / sit / lie / walk / stand / none`
sí puede aprender esa frontera (si los datos la contienen).

### 5. Extensibilidad
- Añadir una clase nueva al detector por reglas = reescribir lógica y
  recalibrar umbrales a mano.
- Añadirla al modelo = etiquetar datos y reentrenar.

Lo segundo escala; lo primero no.

## Encuadre para la defensa

> El detector heurístico es el *baseline* del sistema y demuestra que la señal
> del radar es suficiente para detectar caídas en condiciones controladas. El
> modelo XGBoost es la contribución metodológica del TFM: convierte un sistema
> calibrado manualmente en uno **aprendido a partir de datos**, lo que permite
> (a) cuantificar el rendimiento con métricas estándar, (b) generalizar a
> escenarios fuera del set de calibración, y (c) extender el sistema a nuevas
> clases de actividad sin reescribir lógica.

## Postura defensiva ante objeciones

- **"Pero el detector por reglas ya funciona"** → es el baseline; sin él no hay
  con qué comparar el ML. El ML no lo sustituye, lo complementa (en
  `radar_reader.py` ambos corren en paralelo con `--ml-model`).
- **"¿No es overkill XGBoost para esto?"** → la complejidad está justificada
  por (a) la dimensionalidad del feature space (36) y (b) la necesidad de
  cuantificar rendimiento con LOSO-CV.
- **"¿Por qué no una red neuronal?"** → con un dataset pequeño (sesiones de un
  solo sujeto, hardware único), XGBoost es la elección correcta: alto
  rendimiento con pocas muestras, interpretable vía feature importance,
  desplegable en Raspberry Pi sin GPU. LSTM/CNN están implementados como
  opción futura cuando crezca el dataset.

## Referencias en el código

- Detector por reglas: `tools/radar_reader.py` → clase `FallDetector`.
- Pipeline ML completo: `tools/ml_logger.py`, `tools/feature_engineering.py`,
  `tools/train_model.py`, `tools/evaluate_model.py`, `tools/ml_inference.py`.
- Limitación documentada del baseline: `CLAUDE.md` → sección
  "Known Limitations" → Tier-2 SLOW.
