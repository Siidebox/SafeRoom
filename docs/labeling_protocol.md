# SafeRoom — Protocolo de etiquetado

Versión: 1.0 (2026-05-19). Acompaña `tools/label_session_multimodal.py`.

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
| `fall` | `f` | Caída involuntaria con impacto / colapso al suelo. Desde el primer frame con `vz < 0` claro hasta la última muestra antes de cualquier intento voluntario de levantarse. | Sentarse, tumbarse, agacharse rápido pero controlado. |
| `near_fall` | `a` | Pérdida de equilibrio visible (tropiezo, traspié) sin llegar al suelo. | Si la persona acaba en el suelo → es `fall`, no `near_fall`. |
| `sit` | `s` | Acción de sentarse + permanencia sentado. Comprende la transición. | De pie quieto → eso es `stand`. |
| `lie` | `l` | Persona tumbada de forma voluntaria (cama, sofá, suelo deliberado). | Persona en el suelo tras una caída → `fall` durante ese segmento. |
| `walk` | `w` | Marcha continua. Pasos visibles, desplazamiento neto. | Quieto con micro-movimientos → `stand`. |
| `stand` | `t` | De pie sin desplazarse, balanceo mínimo aceptable. | Empezar a caminar → cortar y etiquetar el desplazamiento como `walk`. |
| `none` | `n` | Habitación vacía. Sin tracks o sólo ghost tracks aislados. | Si entra una persona → cortar y etiquetar lo que haga. |

## Criterios de frontera

- **Inicio de `fall`**: primer frame donde `vz` se vuelve claramente
  negativa (< −0.5 m/s) o donde la persona ha perdido el control vertical.
- **Final de `fall`**: persona inmóvil en el suelo durante ≥ 2 s. A partir
  de ahí, si sigue tumbada y no se levanta, **etiquetar el resto como `fall`**
  hasta que se reincorpore (el modelo aprende también la persistencia post-impacto).
- **`near_fall` vs `walk`**: cuando hay un tropiezo claro (≥ 1 frame con
  `vz` y `az` anómalos) seguido de recuperación. Si la duda es razonable,
  preferir `walk`.
- **Bloques largos `none`**: ok etiquetar 60 s seguidos.
- **Coherencia con el IR confirmer**: el confirmer evalúa los 5 s
  posteriores al evento del radar. Si la etiqueta `fall` cubre ese
  intervalo completo (cuerpo en el suelo), el IR confirmará. Si el
  etiquetador termina la etiqueta antes (p.ej. en el instante del
  impacto), la métrica de fusión penalizará incorrectamente al IR.
  **Regla**: extender `fall` durante al menos 5 s tras el impacto.

## Buenas prácticas

1. **Mirar IR antes de decidir**: en transiciones rápidas el radar puede ser
   ambiguo (cluster pequeño en el suelo = caída o ghost). La cámara IR aclara.
2. **Etiquetar con generosidad** los segmentos de `fall`: incluir el suelo
   posterior. No incluir el levantarse.
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
