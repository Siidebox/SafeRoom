# SafeRoom — Protocolo de etiquetado

Versión: 1.1 (2026-07-08). Acompaña `tools/label_session_multimodal.py`.

Cambios v1.1 (decisiones pre-dataset, revisión metodológica 2026-07-08):
- Nueva etiqueta **`fall_lying`** (tecla `g`): separa el impacto (`fall`) de la
  permanencia en el suelo. El ML entrena solo con `fall`; la métrica de fusión
  IR usa `fall ∪ fall_lying`. Antes un mismo `fall` extendido 5 s servía a dos
  definiciones incompatibles y chocaba con la clase `lie`.
- Regla de ventana: una ventana es positiva solo si contiene **≥ 5 frames
  (250 ms) de `fall`**; ventanas con menos frames de `fall` o con `fall_lying`
  se excluyen del entrenamiento (implementado en `feature_engineering.py`).
- Prohibido usar `fall_detected` (salida de las reglas) como etiqueta:
  `feature_engineering.py` ahora lanza error si falta la columna `label`.

## Reloj y propagación

Todas las etiquetas se almacenan en **dos sitios** consistentes entre sí:

1. `radar.csv` → columna `label` por fila (compatibilidad con el pipeline ML
   actual: `feature_engineering.py` lee esta columna).
2. `manifest.json` → lista `labels` con `(t_start_mono_ns, t_end_mono_ns,
   label, notes)`. Fuente canónica para reanálisis posterior multimodal.

El cursor IR del etiquetador busca el frame térmico más cercano al
`t_mono_ns` del CSV → usa **siempre `t_mono_ns`** como verdad temporal.

## Vocabulario

| Etiqueta | Tecla | Cuándo aplicarla | Cuándo NO |
|---|---|---|---|
| `fall` | `f` | Caída involuntaria con impacto / colapso al suelo. Desde el primer frame con `vz < 0` claro hasta ~2 s después del impacto (persona ya inmóvil en el suelo). | Sentarse, tumbarse, agacharse rápido pero controlado. La permanencia posterior en el suelo → `fall_lying`. |
| `fall_lying` | `g` | Persona en el suelo tras una caída, desde el final de `fall` hasta el primer intento de levantarse. Debe cubrir ≥ 5 s tras el impacto (ventana del IR confirmer). | Tumbarse voluntario → `lie`. El impacto en sí → `fall`. |
| `near_fall` | `a` | Pérdida de equilibrio visible (tropiezo, traspié) sin llegar al suelo. | Si la persona acaba en el suelo → es `fall`, no `near_fall`. |
| `sit` | `s` | Acción de sentarse + permanencia sentado. Comprende la transición. | De pie quieto → eso es `stand`. |
| `lie` | `l` | Persona tumbada de forma voluntaria (cama, sofá, suelo deliberado). | Persona en el suelo tras una caída → `fall` durante ese segmento. |
| `walk` | `w` | Marcha continua. Pasos visibles, desplazamiento neto. | Quieto con micro-movimientos → `stand`. |
| `stand` | `t` | De pie sin desplazarse, balanceo mínimo aceptable. | Empezar a caminar → cortar y etiquetar el desplazamiento como `walk`. |
| `none` | `n` | Habitación vacía. Sin tracks o sólo ghost tracks aislados. | Si entra una persona → cortar y etiquetar lo que haga. |

## Criterios de frontera

- **Inicio de `fall`**: primer frame donde `vz` se vuelve claramente
  negativa (< −0.5 m/s) o donde la persona ha perdido el control vertical.
- **Final de `fall`**: persona inmóvil en el suelo durante ≥ 2 s tras el
  impacto. A partir de ahí, si sigue tumbada y no se levanta, **etiquetar el
  resto como `fall_lying`** hasta que se reincorpore. El levantarse no se
  incluye en ninguna de las dos.
- **`near_fall` vs `walk`**: cuando hay un tropiezo claro (≥ 1 frame con
  `vz` y `az` anómalos) seguido de recuperación. Si la duda es razonable,
  preferir `walk`.
- **Bloques largos `none`**: ok etiquetar 60 s seguidos.
- **Coherencia con el IR confirmer**: el confirmer evalúa los 5 s
  posteriores al evento del radar. La unión `fall + fall_lying` debe cubrir
  ese intervalo completo (cuerpo en el suelo): **`fall_lying` debe durar al
  menos hasta 5 s después del impacto**. `replay_session.py` fusiona
  automáticamente `fall` con su `fall_lying` contiguo al construir los
  eventos ground-truth, así una detección tardía (p. ej. Tier-2 faint
  durante la permanencia en el suelo) cuenta como acierto y no como falsa
  alarma.

## Buenas prácticas

1. **Mirar IR antes de decidir**: en transiciones rápidas el radar puede ser
   ambiguo (cluster pequeño en el suelo = caída o ghost). La cámara IR aclara.
2. **`fall` acotado al impacto, `fall_lying` generoso**: el impacto termina
   ~2 s después de tocar suelo; todo el suelo posterior es `fall_lying`.
   No incluir el levantarse en ninguna de las dos.
3. **`unknown` queda prohibido en el dataset final**: cualquier fila que tras
   etiquetar siga como `unknown` debe convertirse a `none` o descartarse antes
   de entrenar.
4. **No reetiquetar sin documentar**: si cambia el criterio, anotar en
   `manifest.notes` qué se cambió y por qué.
5. **Sanity check post-etiquetado**: ejecutar
   `python tools/feature_engineering.py sessions/<id>/` y verificar que el
   número de ventanas `fall` extraídas es razonable (al menos 1 por sesión de
   caída).

## Casos especiales

- **Múltiples personas**: el sistema no garantiza segmentación por sujeto en
  recogida. Etiquetar la sesión completa por la actividad dominante; si hay
  dos actividades simultáneas, descartar para v1 del dataset.
- **Ghost tracks**: si el ML pipeline confunde un ghost track con persona,
  registrarlo en `manifest.notes`. No se etiqueta el ghost como `fall`.
- **Sesiones cortas (< 10 s)**: válidas para `fall` y `near_fall` siempre que
  se incluyan ≥ 3 s antes del evento. Para clases continuas (`walk`,
  `stand`, `none`) preferir sesiones ≥ 20 s.

## Resultado esperado por sesión

Tras etiquetar y guardar:

- `radar.csv` con `label` ≠ `unknown` en el 100% de las filas con track activo.
- `manifest.json` → `labels` con ≥ 1 `LabelSpan` por sesión.
- Backup `.bak` del CSV se crea automáticamente la primera vez.
