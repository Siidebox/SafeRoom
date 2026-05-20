# SafeRoom — Protocolo de recogida de datos

Versión: 1.0 (2026-05-19). Para uso una vez que la caja v2 esté montada con
el radar IWR6843AOPEVM y la cámara MLX90640 conectados a la Raspberry Pi 5.

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
| Habitación | ~4.5 × 2.5 m | `boundaryBox` del `.cfg` |
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
2. **Iniciar** (la cámara IR auto-calibra los primeros 30 s — la habitación
   debe estar **vacía** durante ese intervalo, después puede entrar el sujeto):
   ```
   python tools/session_recorder.py \
     --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
     --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
     --duration 30 --ir-hz 16 \
     --name <sujeto>_<actividad>_<posicion> \
     --subject <sujeto> \
     --notes "<orientacion>,<ropa>,<hora>"
   ```
   Espera al mensaje `[IR] calibration OK` antes de entrar a la habitación.
   Si ves `[IR] calibration aborted — hot pixels detected`, sal de la
   habitación y espera 60 s — el script reintenta automáticamente.
3. **Ejecutar** la acción siguiendo los tiempos del cronograma:
   - Caídas / near_fall: empezar de pie 5 s, ejecutar, mantener tumbado 5 s.
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

## Cronograma sugerido (un día de captura)

| Bloque | Duración | Contenido |
|---|---|---|
| Setup + precheck | 30 min | Cableado, montaje provisional, smoke test |
| Bloque 1 | 1 h | 30 sesiones de fall (4 orientaciones × variaciones) |
| Descanso | 15 min | |
| Bloque 2 | 45 min | 30 sit + 15 lie + 15 near_fall |
| Bloque 3 | 30 min | 30 walk + 15 stand |
| Bloque 4 | 20 min | 6 × none de 100 s |
| Etiquetado | 1–2 h | Todas las sesiones del día |

Con un segundo sujeto, repetir bloques 1–3 reducidos a la mitad otro día.

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
