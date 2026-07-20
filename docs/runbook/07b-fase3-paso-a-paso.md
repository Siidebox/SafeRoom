# Fase 3 — Grabación paso a paso (bloque por bloque)

Guía autónoma para la grabación del dataset. Sigue los pasos **en orden, de
arriba a abajo**. Todo se ejecuta en la **Raspberry Pi**, desde `~/SafeRoom`.

Antes de nada: pulsa el botón **RST** del radar.

---

## Paso 0 — Definir variables (una sola vez)

Al abrir la terminal en la Pi, define el sujeto y la ruta del `.cfg`. Así no
las repites en cada comando:

```bash
cd ~/SafeRoom
SUJ=guillermo          # <-- cambia por el identificador real del sujeto
CFG=code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg
```

> `$SUJ` y `$CFG` **solo viven en esa terminal**. Si reconectas por SSH o abres
> otra ventana, vuelve a ejecutar estas tres líneas.

---

## Cómo validar (referencia — lo usarás en cada paso)

**A) Validar una sesión normal (fps).** Al terminar cada grabación, el recorder
imprime algo así:

```
[REC] Radar: 600 frames, 0 drops, 20.04 fps
[REC] IR: 240 frames, 0 bad, 7.92 fps
[REC] Manifest written: sessions/<id>/manifest.json
```

La sesión es **válida** si:
- `radar fps >= 19.5`
- `IR fps >= 7.5`
- `drops` ≈ 0  y  `bad` ≈ 0

Si no se cumple → **RST del radar y repite esa sesión**.

**B) Validar una calib.** Después de grabar la calib, el recorder imprime su
carpeta: `[REC] Session: sessions/<timestamp>_calib_bN`. Copia ese id y pégalo
en este comando (sustituye `<PEGA_AQUI_EL_ID_DE_LA_CALIB>`):

```bash
~/SafeRoom/.venv/bin/python -c "import sys; sys.path.insert(0,'tools'); from replay_session import load_background_from_session as l; l('sessions/<PEGA_AQUI_EL_ID_DE_LA_CALIB>'); print('calib OK')"
```

- Imprime `calib OK` → la calib es válida.
- Lanza un error (había alguien o algo caliente en la sala) → **regrábala** con
  la habitación vacía.

**C) Validar la sincronía (cada ~10 sesiones).** Copia el id de una sesión
cualquiera con movimiento:

```bash
~/SafeRoom/.venv/bin/python tools/check_sync.py sessions/<id_de_una_sesion> --max-drift-ms 50
```

---

## Bloque 1 — Caídas (fall)  ·  25–30 sesiones de 30 s

**Paso 1.1 — Calib del bloque 1.** Sal de la habitación (vacía). Grábala y
valídala con el método **B** de arriba:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 60 --ir-hz 16 --name calib_b1 --subject none --notes "calibracion IR bloque 1"
```

> Si ya grabaste `calib_b1` en la Fase 2.5 (smoke tests) y sigue vacía la sala,
> puedes reutilizarla y saltar este paso.

**Paso 1.2 — Grabar las caídas.** Repite este comando **25–30 veces**. Guion de
cada caída: **de pie 5 s → caída → ≥ 8 s inmóvil en el suelo → levantarse.**

Antes de cada repetición, **edita dos cosas** (quita los paréntesis):
- la posición en `--name` → `(centro)` / `(izq)` / `(dcha)` (usa ≥ 3 puntos)
- la orientación en `--notes` → `(frontal)` / `(lat-izq)` / `(lat-dcha)` / `(atras)`

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 30 --ir-hz 16 --name ${SUJ}_fall_(centro) --subject $SUJ --notes "(frontal)"
```

**Tras cada sesión:** mira los fps (método **A**). Si una caída sale rara (te
frenas, media caída), repítela.

---

## Bloque 2 — Vida normal negativa  ·  1 sesión de ~1200 s (20 min)

**Paso 2.1 — Calib del bloque 2** (sala vacía) + validar (método B):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 60 --ir-hz 16 --name calib_b2 --subject none --notes "calibracion IR bloque 2"
```

**Paso 2.2 — Grabar la sesión negativa.** Vida normal **SIN caídas**, metiendo
confusores: dejar caer una mochila/objeto, agacharte a recoger, sentarte
brusco, mover una silla, abrir/cerrar la puerta.

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 1200 --ir-hz 16 --name ${SUJ}_negativo_1 --subject $SUJ
```

Valida los fps al terminar (método A).

---

## Bloque 3 — Casi-caídas y sentarse

**Paso 3.1 — Calib del bloque 3** (sala vacía) + validar (método B):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 60 --ir-hz 16 --name calib_b3 --subject none --notes "calibracion IR bloque 3"
```

**Paso 3.2 — near_fall  ·  15 sesiones de 20 s.** Guion: de pie 5 s → tropiezo
→ recuperarse **sin tocar el suelo**. Cambia la orientación en `--notes`:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 20 --ir-hz 16 --name ${SUJ}_nearfall_(centro) --subject $SUJ --notes "(frontal)"
```

**Paso 3.3 — sit  ·  20 sesiones de 20 s** (10 en silla + 10 en el suelo).
Guion: de pie 3 s → sentarse → quieto 10 s. Cambia `(silla)` por `(suelo)` a
partir de la sesión 11:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 20 --ir-hz 16 --name ${SUJ}_sit_(silla) --subject $SUJ
```

Valida los fps tras cada sesión (método A).

---

## Bloque 4 — Caminar, tumbarse, de pie, vacío

**Paso 4.1 — Calib del bloque 4** (sala vacía) + validar (método B):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 60 --ir-hz 16 --name calib_b4 --subject none --notes "calibracion IR bloque 4"
```

**Paso 4.2 — walk  ·  15–20 sesiones de 30 s.** Caminar continuo en distintas
direcciones. **Incluye caminar perpendicular al radar** (cambia `(perpendicular)`
por `(frontal)` / `(diagonal)`):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 30 --ir-hz 16 --name ${SUJ}_walk_(perpendicular) --subject $SUJ
```

**Paso 4.3 — lie  ·  10 sesiones de 25 s.** Guion: de pie 3 s → tumbarse
voluntario → quieto 10 s. Cambia la posición `(centro)`:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 25 --ir-hz 16 --name ${SUJ}_lie_(centro) --subject $SUJ
```

**Paso 4.4 — stand  ·  10 sesiones de 20 s.** Quieto de pie; cambia de sitio
entre sesiones `(centro)`:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 20 --ir-hz 16 --name ${SUJ}_stand_(centro) --subject $SUJ
```

**Paso 4.5 — none  ·  2 sesiones de 100 s.** Habitación **vacía** (sal tú).
Cambia el número `(1)` → `(2)`:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 100 --ir-hz 16 --name none_(1) --subject none
```

Valida los fps tras cada sesión (método A).

---

## Bloque 5 — Segunda negativa y secuencias continuas

**Paso 5.1 — Calib del bloque 5** (sala vacía) + validar (método B):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 60 --ir-hz 16 --name calib_b5 --subject none --notes "calibracion IR bloque 5"
```

**Paso 5.2 — negativo 2  ·  1 sesión de ~1200 s.** Otra sesión de vida normal
sin caídas (a otra hora del día si puedes):

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 1200 --ir-hz 16 --name ${SUJ}_negativo_2 --subject $SUJ
```

**Paso 5.3 — continuo  ·  2 sesiones de 300 s.** Secuencia completa con caída:
entrar → caminar → de pie → sentarse → caminar → **caída** → suelo ≥ 10 s →
levantarse → salir. Cambia el número `(1)` → `(2)`:

```bash
~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg $CFG --duration 300 --ir-hz 16 --name ${SUJ}_continuo_(1) --subject $SUJ
```

Valida los fps tras cada sesión (método A).

---

## Recordatorios

- **Quita siempre los paréntesis** al poner el valor real:
  `${SUJ}_fall_(centro)` → `${SUJ}_fall_centro`. El nombre de sesión no lleva
  espacios ni paréntesis.
- Si ves `Dropping frame` o los fps bajan → **RST del radar** y repite.
- Cada ~10 sesiones, corre la validación de sincronía (método C).
- Las sesiones `calib_bN` **no se etiquetan** (son solo fondo térmico para el
  confirmador IR). El resto sí se etiqueta después en el PC (Fase 6).
- Al acabar el día, copia `sessions/` al PC (Fase 5 de la guía completa) y haz
  una copia de seguridad aparte.
