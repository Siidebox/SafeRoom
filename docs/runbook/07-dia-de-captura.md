# Día de captura del dataset — guía completa

Todos los pasos del día, en orden, de principio a fin. Grabación en la
**Raspberry Pi** (`~/SafeRoom`, venv `~/SafeRoom/.venv`), etiquetado y
entrenamiento después en el **PC Windows** (repo en
`C:\Users\sideb\chicago\IIT\tfm\SafeRoom`).

Protocolo de referencia: `docs/data_collection_protocol.md` (v1.1, mínimo
viable). Criterios de etiquetado: `docs/labeling_protocol.md` (v1.1).

---

## Fase 0 — Preparar la Pi (10 min)

```bash
ssh guillermo@piSafeRoom.local
cd ~/SafeRoom && git pull
ls /dev/ttyUSB*          # deben aparecer /dev/ttyUSB0 (CLI) y /dev/ttyUSB1 (datos)
df -h ~/SafeRoom         # comprueba espacio libre: un día (~3.5 h con IR) ocupa varios GB
```

- Pulsa el botón **RST** del IWR6843AOPEVM (siempre antes de empezar).
- Comprobación rápida del entorno (opcional pero recomendada):

```bash
~/SafeRoom/.venv/bin/python -m pytest tests/ -q
```

## Fase 1 — Montaje físico (30–45 min)

1. Sensor a **2.04 m de altura, tilt 10° hacia abajo**, misma posición que
   `sensorPosition` en `SafeRoom_1p9m_4x6m.cfg`: en la pared corta (3.7 m),
   desplazado **1.30 m desde la esquina** (no centrado) — lado estrecho
   (1.30 m) a la izquierda del radar, lado ancho (2.40 m) a la derecha. El
   `boundaryBox` del `.cfg` ya refleja esta asimetría en X (−1.30 / +2.40).
2. MLX90640 junto al radar, FOV alineado. Montaje actual: **90°**, que ya es
   el valor por defecto de `--ir-rotate` en live y en replay (se puede omitir
   el flag). Verificado visualmente sobre las sesiones del 2026-07-21 (la
   persona queda bien colocada con `--ir-rotate 90`). Si cambias el montaje
   físico, **anota la nueva rotación**
   (0/90/180/270°) y pásala explícitamente con `--ir-rotate` en ambos
   comandos — deben coincidir, porque el confirmador IR se calibra en
   coordenadas post-rotación.
3. Cableado a la Pi; comprueba que los dos puertos USB siguen visibles.
4. Suelo despejado (sin objetos > 0.5 m dentro del `boundaryBox`).
5. Colchoneta colocada en las zonas de caída.
6. Consentimiento informado del sujeto firmado (aunque seas tú mismo,
   deja constancia escrita para la memoria del TFM).

## Fase 2 — Smoke tests (20–30 min)

Desde `~/SafeRoom` en la Pi. **RST del radar antes de cada intento fallido.**

**2.1 Precheck de grabación (10 s):**

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --duration 10 --ir-hz 16 --name precheck
```

Verifica en la salida: `radar fps >= 19.5`, `IR fps >= 7.5`, `drops` ~0.
Si falla → runbook `06-troubleshooting.md`.

**2.2 Verificación visual del track** (con pantalla o X forwarding):

```bash
~/SafeRoom/.venv/bin/python tools/radar_reader.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --plot --sensor-height 2.04 --sensor-tilt 10.0
```

> No omitas `--sensor-height 2.04 --sensor-tilt 10.0`: los defaults del script
> (`2.05` / `15.0`) están desfasados respecto a la geometría real de la sala.

Camina por la habitación: el track debe seguirte dentro del `boundaryBox` y
`maxZ` debe ser plausible de pie (~1.2–1.7 relativo). Cruza también
perpendicular al radar y anota si el track se pierde (limitación Doppler
conocida — documentar, no bloquea).

**Comprobación específica de la asimetría en X (habitación nueva):** ve hasta
la pared del lado ancho y confirma que el track llega hasta X ≈ +2.40 sin
recortarse; ve hasta la pared del lado estrecho y confirma X ≈ −1.30. Si el
track se recorta antes de llegar a la pared o el signo sale invertido (el
lado ancho aparece en −X), el `boundaryBox` tiene el signo de X equivocado —
corrígelo en el `.cfg` y reenvía la configuración antes de seguir.

**2.3 Ensayo de caída (1 vez, sobre colchoneta):** con el mismo comando
anterior corriendo, simula una caída. Debe aparecer la alerta Tier-1
(vz ≤ −1.15 m/s). Si no dispara, anótalo — no bloquea la grabación (las
etiquetas son post-hoc), pero revisa que la caída fue franca.

**2.4 Sincronía radar↔IR** (usa la sesión precheck):
/home/guillermo/SafeRoom/sessions/20260720_103819_synccheck
```bash
~/SafeRoom/.venv/bin/python tools/check_sync.py sessions/20260720_103819_synccheck --max-drift-ms 50 --plot
```

**2.5 Calibración IR del bloque 1** (60 s, habitación VACÍA — sal tú también):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --duration 60 --ir-hz 16 --name calib_b1 --subject none --notes "calibracion IR bloque 1"
```
/home/guillermo/SafeRoom/sessions/20260720_104243_calib_b1
Validarla en el momento:

```bash
~/SafeRoom/.venv/bin/python -c "import sys; sys.path.insert(0,'tools'); from replay_session import load_background_from_session as l; l('sessions/20260720_104243_calib_b1'); print('calib OK')"
```

Si da error (alguien estaba dentro), regrábala. **Listo para grabar.**

## Fase 3 — Grabación por bloques (~3.5 h)

Cada actividad tiene su comando abajo, listo para copiar. Los directorios de
sesión se nombran `<timestamp>_<name>`, así que el nombre solo lleva
`<sujeto>_<actividad>_<posicion>` (el timestamp lo pone el recorder).

**Define el sujeto una vez** al empezar el día (evita repetirlo en cada
comando):

```bash
cd ~/SafeRoom
SUJ=guillermo          # cambia por el identificador del sujeto real
CFG=code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg
```

La variable `$SUJ` y `$CFG` solo viven en esa terminal — si abres otra o
reconectas por SSH, vuelve a definirlas.

**Al inicio de cada bloque nuevo (~1 h): calib de 60 s con la sala vacía**
(cambia `bN` por el número de bloque) y valídala:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 60 --ir-hz 16 --name calib_b1 --subject none --notes "calibracion IR bloque 1"
~/SafeRoom/.venv/bin/python -c "import sys; sys.path.insert(0,'tools'); from replay_session import load_background_from_session as l; l('sessions/<id_calib_bN>'); print('calib OK')"
```

Tabla-resumen (el guion de cada sesión; los comandos van justo debajo):

| Bloque | Sesiones | `--duration` | Guion de cada sesión |
|---|---|---|---|
| B1 | 25–30 × fall | 30 | De pie 5 s → caída → **≥ 8 s inmóvil en el suelo** → levantarse. Rotar orientación (frontal / lat-izq / lat-dcha / atrás) y posición (≥ 3 puntos). |
| B2 | 1 × negativo | 900–1800 | Vida normal SIN caídas, incluyendo confusores: dejar caer mochila/objeto, agacharse a recoger, sentarse brusco, mover silla, abrir/cerrar puerta. |
| B3 | 15 × near_fall | 20 | De pie 5 s → tropiezo → recuperarse sin tocar suelo. |
| B3 | 20 × sit | 20 | De pie 3 s → sentarse (10 en silla, 10 en suelo) → quieto 10 s. |
| B4 | 15–20 × walk | 30 | Caminar continuo, distintas direcciones (incluir perpendicular). |
| B4 | 10 × lie | 25 | De pie 3 s → tumbarse voluntario → quieto 10 s. |
| B4 | 10 × stand | 20 | Quieto de pie; cambiar de posición entre sesiones. |
| B4 | 2 × none | 100 | Habitación vacía. |
| B5 | 1 × negativo | 900–1800 | Segunda sesión negativa, distinta hora/ropa si es posible. |
| B5 | 2 × continuo | 300 | Entrar → caminar → de pie → sentarse → caminar → **caída** → suelo ≥ 10 s → levantarse → salir. |

En cada comando edita lo que va **entre paréntesis** (posición, orientación,
número de repetición) antes de lanzarlo.

**B1 — fall (25–30 sesiones, 30 s):** rota `pos` (centro/izq/dcha, ≥ 3 puntos)
y la orientación en `--notes` (frontal/lat-izq/lat-dcha/atras):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 30 --ir-hz 16 --name ${SUJ}_fall_(centro) --subject $SUJ --notes "(frontal),(camiseta_algodon),(tarde)"
```

**B2 — negativo (1 sesión larga, ~1200 s):** vida normal con confusores:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 1200 --ir-hz 16 --name ${SUJ}_negativo_1 --subject $SUJ --notes "vida_normal_confusores,(camiseta_algodon),(tarde)"
```

**B3 — near_fall (15 sesiones, 20 s):** tropiezo sin tocar suelo:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 20 --ir-hz 16 --name ${SUJ}_nearfall_(centro) --subject $SUJ --notes "(frontal),(camiseta_algodon),(tarde)"
```

**B3 — sit (20 sesiones, 20 s):** cambia `silla`→`suelo` a mitad (10 y 10):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 20 --ir-hz 16 --name ${SUJ}_sit_(silla) --subject $SUJ --notes "(camiseta_algodon),(tarde)"
```

**B4 — walk (15–20 sesiones, 30 s):** rota `dir` (frontal/perpendicular/diag):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 30 --ir-hz 16 --name ${SUJ}_walk_(perpendicular) --subject $SUJ --notes "(camiseta_algodon),(tarde)"
```

**B4 — lie (10 sesiones, 25 s):** tumbarse voluntario:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 25 --ir-hz 16 --name ${SUJ}_lie_(centro) --subject $SUJ --notes "(camiseta_algodon),(tarde)"
```

**B4 — stand (10 sesiones, 20 s):** quieto de pie, cambia de sitio:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 20 --ir-hz 16 --name ${SUJ}_stand_(centro) --subject $SUJ --notes "(camiseta_algodon),(tarde)"
```

**B4 — none (2 sesiones, 100 s):** habitación vacía (sujeto `none`, sal tú):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 100 --ir-hz 16 --name none_(1) --subject none --notes "habitacion_vacia,(tarde)"
```

**B5 — negativo 2 (1 sesión, ~1200 s):** otra hora/ropa si puedes:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 1200 --ir-hz 16 --name ${SUJ}_negativo_2 --subject $SUJ --notes "vida_normal_confusores,(sudadera_sintetica),(noche)"
```

**B5 — continuo (2 sesiones, 300 s):** secuencia completa con caída incluida:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 300 --ir-hz 16 --name ${SUJ}_continuo_(1) --subject $SUJ --notes "entrar-caminar-caida-suelo-salir,(camiseta_algodon),(noche)"
```

> Los paréntesis son marcadores para editar — **quítalos** al poner el valor
> real (`${SUJ}_fall_(centro)` → `${SUJ}_fall_centro`). El nombre de sesión no
> lleva espacios; las `--notes` sí pueden, por eso van entre comillas.

Consejos:
- Cada sesión imprime sus fps al terminar — **mira los quality gates antes
  de pasar a la siguiente** (Fase 4).
- Varía ropa al menos una vez durante el día (algodón vs sintético).
- Si una caída sale rara (te frenas, media caída), repítela: es más barato
  ahora que descartarla en etiquetado.
- El radar necesita RST si ves `Dropping frame` o el precheck baja de fps.

## Fase 4 — Quality gates (tras cada sesión)

Repetir la sesión si:
- `radar.fps_real < 19.5`
- `thermal.fps_real < 7.5`
- `radar.drops > 1 %` de frames
- `thermal.bad_frames > 1 %` de frames

Cada ~10 sesiones, comprueba sincronía: `check_sync.py sessions/<id> --max-drift-ms 50`.

## Fase 5 — Copiar las sesiones al PC (10 min)

Desde **PowerShell en el PC**, en la raíz del repo:

```powershell
scp -r guillermo@piSafeRoom.local:~/SafeRoom/sessions/* .\sessions\
```

(Si `piSafeRoom.local` no resuelve, usa la IP de Tailscale.)
`sessions/` está en `.gitignore` — los datos no se suben a git; haz copia
de seguridad aparte (disco externo / Drive) al terminar el día.

## Fase 6 — Etiquetado (en el PC, 1–2 h)

Por cada sesión (empieza por las de fall):

```powershell
python tools\label_session_multimodal.py sessions\<id>\
```

Interacción: arrastra en el plot para seleccionar un rango temporal y pulsa
la tecla de la etiqueta. `z` deshace, `Enter` guarda (CSV + manifest),
`Esc` sale sin guardar.

| Tecla | Etiqueta | Recuerda |
|---|---|---|
| `f` | fall | Del primer frame con vz < −0.5 claro hasta ~2 s tras el impacto. |
| `g` | fall_lying | Desde el fin de `fall` hasta que empieza a levantarse. Debe cubrir ≥ 5 s tras el impacto. |
| `a` | near_fall | Tropiezo sin llegar al suelo. |
| `s` | sit | Transición + permanencia sentado. |
| `l` | lie | Tumbarse voluntario (≠ suelo tras caída). |
| `w` | walk | Marcha con desplazamiento neto. |
| `t` | stand | De pie sin desplazarse. |
| `n` | none | Habitación vacía / sin persona. |

Criterios completos y casos dudosos: `docs/labeling_protocol.md`.
Las sesiones `calib_bN` no se etiquetan (son solo fondo térmico).

Sanity check tras etiquetar todo:

```powershell
python tools\feature_engineering.py sessions\<una_sesion_de_fall>\
```

Debe extraer ≥ 1 ventana `fall` por sesión de caída.

## Fase 7 — Entrenar y evaluar (en el PC, ~1 h)

**7.1 Entrenar.** Primero saca las sesiones `calib_bN` fuera de `sessions\`
(solo son fondo térmico; entrenar con ellas mete un grupo LOSO de puro `none`).
Los directorios se llaman `<timestamp>_calib_bN`, así que muévelos por sufijo:

```powershell
mkdir sessions_calib
move sessions\*_calib_* sessions_calib\
python tools\train_model.py --data sessions\*\ --no-dl
```

Salida: `models/fall_detector_xgb.pkl`, `models/fall_detector_rf.pkl`,
`models/cv_results.csv`. Ojo: si el threshold óptimo vuelve a caer en el
borde de la rejilla (0.10), es señal de sobreajuste — revisar.

**7.2 Figuras de evaluación:**

```powershell
python tools\evaluate_model.py --data sessions\*\ --no-dl --outdir figures\eval --latex
```

**7.3 Tabla comparativa reglas vs ML vs fusión (la tabla del TFM)** —
replay offline pareado sobre las mismas sesiones, con métricas de evento
(recall/evento, falsas alarmas/hora, latencia):

```powershell
python tools\replay_session.py sessions\*_<sujeto>_* --ml-model models\fall_detector_xgb.pkl --ir-calib sessions\<calib_b1_id> --ir-rotate 90 --json figures\replay_results.json
```

> El glob es `sessions\*_<sujeto>_*` (no `sessions\<sujeto>_*`): los directorios
> empiezan por el timestamp (`<YYYYMMDD_HHMMSS>_<sujeto>_...`), así que el sujeto
> va en medio. Usa `sessions\*` para replay sobre todas las sesiones.

Usa la calib del bloque correspondiente a cada tanda si las condiciones
térmicas cambiaron (sol, calefacción). Revisa en la salida las decisiones
del IR (`confirmed`/`vetoed`/`failopen`) sobre cada caída — los `vetoed`
sobre caídas reales son el análisis de fail-closed pendiente del TFM.

**7.4 Estudio A_kinetic** (con datos reales, cierra el gate ROC AUC ≥ 0.7):

```powershell
python tools\explore_ir_kinetics.py sessions\ --out figures\ir_kinetics
```

## Fase 8 — Si algo falla

| Síntoma | Acción |
|---|---|
| No aparecen `/dev/ttyUSB*` | Recablear USB, `dmesg | tail`, RST |
| `Dropping frame` / fps bajos | RST del radar y repetir sesión |
| IR init failed | Revisar I2C (`i2cdetect -y 1` → 0x33), conexiones MLX |
| `No module named 'board'` / `adafruit_mlx90640` | Venv incorrecto (usa `~/SafeRoom/.venv`) o falta la lib; revisar I2C (`i2cdetect -y 1` → 0x33) |
| Disco lleno a mitad de día | `df -h`; borrar prechecks y sesiones fallidas (`rm -r sessions/<id>`) |
| calib rechazada | Había alguien/fuente de calor; regrabar con habitación vacía |
| Track no aparece en `--plot` | Revisar cfg enviado, boundaryBox, RST |
| Más casos | `docs/runbook/06-troubleshooting.md` |

---

**Resumen del día en una línea:** pull + RST → montar a 2.04 m/10° →
smoke tests → calib 60 s vacía por bloque → grabar B1–B5 vigilando fps →
scp al PC → etiquetar (`f`/`g` bien separados) → entrenar → replay con
`--ir-calib` para la tabla final.
