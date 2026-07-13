# SafeRoom — Protocolo de recogida de datos

Versión: 1.1 (2026-07-08). Para uso una vez que la caja v2 esté montada con
el radar IWR6843AOPEVM y la cámara MLX90640 conectados a la Raspberry Pi 5.

Cambios v1.1: se añade el **protocolo mínimo viable** (1 sujeto, 1 día) como
objetivo del primer día de captura, sesiones negativas no guionizadas con
confusores, sesiones de escenario continuo, y la etiqueta `fall_lying`
(ver `labeling_protocol.md` v1.1). El reparto completo v1.0 pasa a ser el
objetivo extendido si hay más días/sujetos.

## Protocolo mínimo viable (primer día, 1 sujeto)

Subconjunto suficiente para un capítulo de resultados honesto. Prioridad de
mayor a menor — si el día se acorta, cortar desde abajo:

| Prioridad | Clase | Repeticiones | Notas |
|---|---|---|---|
| 1 | `fall` | 25–30 | 4 orientaciones (frontal, lateral izq/dcha, atrás), ≥ 3 posiciones. Colchoneta. |
| 2 | **negativos no guionizados** | 2 sesiones × 15–30 min | Ver abajo. Sin guion, con confusores. |
| 3 | `near_fall` | 15 | |
| 4 | `sit` | 20 | Mitad silla, mitad suelo. |
| 5 | `walk` | 15–20 | Incluir trayectorias perpendiculares al beam. |
| 6 | `lie` | 10 | |
| 7 | `stand` | 10 | |
| 8 | `none` | 2 × 100 s | |
| 9 | **escenario continuo** | 2 sesiones × ~5 min | Ver abajo. |

**Sesiones negativas no guionizadas (prioridad 2).** Vida normal en la
habitación SIN caídas, incluyendo deliberadamente confusores que pueden
disparar falsos positivos del radar: dejar caer una mochila/objeto pesado,
agacharse a recoger algo del suelo, sentarse bruscamente, abrir/cerrar la
puerta, mover una silla, estirarse en el suelo a por algo bajo un mueble.
Son las sesiones que dan potencia estadística a la métrica de falsas
alarmas/hora y al estudio de fusión radar+IR — sin ellas, con solo
actividades guionizadas cortas, el radar puede dar 0 FPs y la comparación
queda vacía. Etiquetarlas con sus clases normales (`walk`, `sit`, `none`...).

**Sesiones de escenario continuo (prioridad 9).** Secuencia mixta en una sola
sesión: entrar → caminar → estar de pie → sentarse → caminar → caída →
permanecer en el suelo ≥ 10 s → levantarse → salir. Sirven como conjunto de
validación realista: en el resto del dataset cada sesión contiene una sola
clase, y los folds LOSO degeneran (el fold de `fall` casi no tiene negativos).

**Sesgo de sujeto único.** Con 1 sujeto, la validación es
leave-one-session-out, no leave-one-subject-out: las claims de generalización
se limitan a "sesiones no vistas del mismo sujeto y entorno". Registrar el
sesgo en `manifest.notes` y en la sección de limitaciones del TFM. El segundo
sujeto (más adelante) repetirá los bloques 1–5 reducidos a la mitad.

## Objetivo

Generar un dataset etiquetado, **multimodal sincronizado** (radar + IR), que
permita entrenar y evaluar de forma rigurosa el modelo de detección de caídas
del TFM. El modelo principal es radar-only; la IR se graba siempre como
ground-truth visual y como baseline secundaria.

## Setup físico

| Elemento | Valor objetivo | Notas |
|---|---|---|
| Altura sensor | 2.04 m | Coincide con `sensorPosition` del `.cfg` activo |
| Tilt | 10° hacia abajo | Validado el 2026-04-20 |
| Habitación | 5.10 × 3.7 m | `boundaryBox` del `.cfg` (X asimétrico: sensor a 1.30 m de la esquina) |
| Suelo | Limpio, sin clutter > 0.5 m | Evita reflectores estáticos |
| MLX90640 | Junto al radar, FOV alineado | Confirmar 16 Hz refresh |
| Iluminación | Indiferente (IR) | Para vídeo de referencia conviene luz natural |

Antes de grabar:
- Pi alimentada y arrancada.
- `sensorStop` enviado y `flushCfg`. Cada sesión envía su `.cfg` al inicio.
- Comprobación rápida con `python tools/session_recorder.py --duration 10
  --name precheck`. Verifica `radar.fps_real ≥ 19.5`, `thermal.fps_real ≥ 7.5`.

## Estructura de salida

Cada sesión produce un directorio:

```
sessions/<YYYYMMDD_HHMMSS>_<sujeto>_<actividad>/
├── radar.csv      (MlCsvLogger, 35 columnas, incluye t_mono_ns)
├── thermal.npz    (MLX90640 24×32 float32 + t_mono_ns + t_wall)
└── manifest.json  (metadatos + LabelSpans tras etiquetado)
```

## Clases y reparto objetivo

Mínimos antes de considerar el dataset entrenable.

| Clase | Definición operativa | Repeticiones objetivo |
|---|---|---|
| `fall` | Caída completa desde de pie hasta suelo, sin volver a levantarse en ≥ 3 s. Frontal, lateral izquierda, lateral derecha, hacia atrás (~25% cada una). | 60 |
| `near_fall` | Tropiezo / pérdida de equilibrio recuperada antes de tocar suelo. | 30 |
| `sit` | Sentarse controladamente en suelo o silla. | 60 |
| `lie` | Tumbarse voluntariamente (cama / colchoneta). | 30 |
| `walk` | Caminar por la habitación, distintas direcciones. | 60 |
| `stand` | De pie quieto en distintas posiciones. | 30 |
| `none` | Habitación vacía. | 600 s acumulados (~6 sesiones × 100 s) |

Total estimado: ~35 min de actividad + ~10 min vacía = **45 min** netos.

## Variación obligatoria

Para que el modelo generalice, cada clase debe cubrir:

- **Posición** en la habitación: ≥ 4 puntos (esquinas + centro).
- **Orientación** respecto al radar: 0°, 45°, 90°, 135°, 180°.
- **Sujetos**: idealmente ≥ 2 personas (alturas / complexiones distintas).
- **Ropa**: al menos una ropa "absorbente" (algodón grueso) y una "reflectiva"
  (sintético / chaqueta).
- **Hora del día**: ≥ 2 momentos distintos (la IR es sensible a temperatura
  ambiente y luz solar).

Si no hay ≥ 2 sujetos disponibles, registrar este sesgo en `manifest.notes`
de cada sesión y mencionarlo en la sección de limitaciones del TFM.

## Procedimiento por sesión

Sesión = una grabación continua de **una sola actividad** (no mezclar clases
en la misma sesión salvo pruebas específicas).

1. **Decidir** clase, sujeto, posición, orientación.
2. **Iniciar**:
   ```
   python tools/session_recorder.py \
     --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
     --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
     --duration 30 --ir-hz 16 \
     --name <sujeto>_<actividad>_<posicion> \
     --subject <sujeto> \
     --notes "<orientacion>,<ropa>,<hora>"
   ```
   `session_recorder.py` solo graba — **no calibra la IR** (eso lo hace el
   `IrConfirmer` en modo live). Para el análisis offline, la calibración
   térmica sale de una **sesión `calib` dedicada**: al inicio de cada bloque
   de grabación (~1 h), graba 60 s con la habitación **vacía**:
   ```
   python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
     --cfg ... --duration 60 --ir-hz 16 --name calib_bloqueN --subject none \
     --notes "calibracion IR bloque N"
   ```
   Después, `replay_session.py --ir-calib sessions/<calib_id>` construye el
   fondo térmico desde ella. Si alguien estuvo en la habitación durante la
   calib, el replay la rechazará con un error claro — regrabarla. El resto
   de sesiones NO necesitan empezar con la habitación vacía.
3. **Ejecutar** la acción siguiendo los tiempos del cronograma:
   - Caídas: empezar de pie 5 s, ejecutar, **permanecer tumbado ≥ 8 s**
     (el `fall_lying` debe cubrir los 5 s de ventana del IR confirmer).
   - Near_fall: empezar de pie 5 s, ejecutar el tropiezo, recuperarse.
   - Sit / lie: de pie 3 s → sentarse / tumbarse → quieto 10 s.
   - Walk: caminar continuo 30 s.
   - Stand: quieto en posición; cambiar posición cada 10 s entre sesiones.
   - None: nadie en la habitación durante toda la sesión.
4. **Verificar** al terminar: el script imprime `radar.fps_real` y `thermal.fps_real`.
   Repetir si:
   - `radar.fps_real < 19.5`,
   - `thermal.fps_real < 7.5`,
   - `radar.drops > 1% × frames`,
   - `thermal.bad_frames > 1% × frames`.
5. **Etiquetar** después (no en caliente):
   ```
   python tools/label_session_multimodal.py sessions/<id>/
   ```

## Cronograma sugerido (día de captura, protocolo mínimo viable)

Cada bloque empieza con una **sesión `calib` de 60 s con la habitación
vacía** (fondo térmico para `replay_session.py --ir-calib`).

| Bloque | Duración | Contenido |
|---|---|---|
| Setup + precheck | 45 min | Montaje sensor en posición (2.04 m, 10°), cableado a la Pi, smoke test (ver checklist abajo) |
| Bloque 1 | 1 h | calib 60 s + 25–30 fall (4 orientaciones × ≥ 3 posiciones) |
| Descanso | 15 min | |
| Bloque 2 | 30–45 min | calib 60 s + negativos no guionizados (1ª sesión larga con confusores) |
| Bloque 3 | 30 min | calib 60 s + 15 near_fall + 20 sit |
| Bloque 4 | 30 min | 15–20 walk + 10 lie + 10 stand + 2 × none 100 s |
| Bloque 5 | 30 min | calib 60 s + 2ª sesión negativa + 2 escenarios continuos |
| Etiquetado | 1–2 h | Todas las sesiones del día (puede ser otro día) |

Con un segundo sujeto (más adelante), repetir bloques 1–3 reducidos a la
mitad otro día.

## Checklist pre-captura (día de setup)

1. Sensor montado a **2.04 m, tilt 10°**, misma posición que `sensorPosition`
   del `.cfg` activo (esquina, tripode o soporte definitivo).
2. MLX90640 junto al radar con FOV alineado; **anotar la rotación de
   montaje** (0/90/180/270°) — debe pasarse igual en `radar_reader.py
   --ir-rotate` (live) y en `replay_session.py --ir-rotate` (offline).
3. Pi arrancada; radar en `/dev/ttyUSB0` (CLI) y `/dev/ttyUSB1` (datos).
4. Smoke test: `python tools/session_recorder.py --duration 10 --name precheck`
   → verificar `radar.fps_real ≥ 19.5` y `thermal.fps_real ≥ 7.5`.
5. Verificación visual con `radar_reader.py --plot`: caminar por la
   habitación y comprobar que el track sigue a la persona dentro del
   `boundaryBox`; comprobar maxZ ≈ altura esperada de pie.
6. Grabar la sesión `calib` del bloque 1 (60 s, habitación vacía) y
   validarla en el momento:
   ```
   python -c "import sys; sys.path.insert(0,'tools'); from replay_session import load_background_from_session as l; l('sessions/<calib_id>'); print('calib OK')"
   ```
   Si alguien estaba en la habitación, dará error con el motivo — regrabar.
7. Ensayo de 1 caída sobre colchoneta con `--plot`: confirmar que Tier-1
   dispara (o anotar si no, para revisar umbral).
8. Colchoneta colocada; consentimiento informado del sujeto firmado
   (aunque sea el propio autor, dejar constancia escrita para el TFM).

## Calidad mínima del dataset

Después de etiquetar, comprobar:

- Cada clase tiene ≥ las repeticiones objetivo arriba.
- Ninguna sesión es la única instancia de su clase para un sujeto dado
  (necesario para LOSO-CV con generalización inter-sujeto).
- Distribución de duraciones (en `manifest.duration_s`) sin outliers
  extremos: ninguna < 5 s para fall.

Si todo OK, comprometer con `git`:

```
git add sessions/<id>/
git commit -m "data: <sujeto> session <YYYYMMDD>, +N falls / +M sit / ..."
```

(No subir vídeos a remoto sin consentimiento. Por defecto sólo CSV + npz.)

## Limitaciones conocidas a documentar en el TFM

- Movimiento lateral perpendicular al beam → bajo Doppler.
- `maxZ − minZ` es extensión del cluster, no estatura.
- IR no etiqueta automáticamente: la etiqueta sigue siendo manual y depende
  del criterio del operador (ver `labeling_protocol.md`).
- 16 Hz IR → ~8 fps efectivos (subpaging) → resolución temporal IR ≈ 125 ms.
- Sin diversidad real de sujetos los modelos sobreajustarán al operador.
