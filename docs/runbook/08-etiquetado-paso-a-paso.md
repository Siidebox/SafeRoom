# Fase 6 — Etiquetado paso a paso (en el PC)

Guía autónoma para etiquetar el dataset una vez grabado y copiado al PC. Sigue
los pasos **en orden**. Todo se ejecuta en el **PC Windows**, en el repo
`C:\Users\sideb\chicago\IIT\tfm\SafeRoom`, desde **PowerShell**.

> El `python` global del PC **no tiene** `pandas` ni las librerías de ML. Por eso
> aquí se usa siempre el intérprete del venv: **`.\.venv\Scripts\python.exe`**.
> Si escribes `python` a secas, la herramienta fallará con
> `ModuleNotFoundError: No module named 'pandas'`.

---

## Paso 0 — Preparar el entorno (una sola vez)

Abre PowerShell en la raíz del repo y crea el venv con las dependencias. **Solo
hace falta la primera vez**; si ya existe `.\.venv`, salta este paso.

```powershell
cd C:\Users\sideb\chicago\IIT\tfm\SafeRoom
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install numpy pandas matplotlib pyserial pytest xgboost scikit-learn joblib
```

Comprueba que quedó bien:

```powershell
.\.venv\Scripts\python.exe -c "import numpy,pandas,matplotlib,xgboost,sklearn,joblib; print('deps OK')"
```

Debe imprimir `deps OK`.

---

## Paso 1 — Limpiar las sesiones que NO son dataset

En `sessions\` hay carpetas que **no** forman parte del dataset y ensuciarían el
entrenamiento. Sácalas fuera (los `calib_*` **NO** se tocan aquí — se mueven más
tarde, en la Fase 7):

```powershell
New-Item -ItemType Directory -Force sessions_excluidas | Out-Null
foreach ($d in @('*precheck','*synccheck','synth_a','synth_b','synth_demo')) {
  Get-ChildItem "sessions\$d" -Directory -ErrorAction SilentlyContinue | Move-Item -Destination sessions_excluidas\
}
```

Qué son y por qué se sacan:
- `*_precheck` → smoke test de 10 s de la Fase 2, no es dato.
- `*_synccheck` → prueba de sincronía radar↔IR, no es dato.
- `synth_a` / `synth_b` / `synth_demo` → sesiones sintéticas de prueba del código.

---

## Paso 2 — Qué se etiqueta y qué no

| ¿Se etiqueta? | Sesiones |
|---|---|
| **SÍ** | `fall`, `nearfall`, `sit`, `walk`, `lie`, `stand`, `none`, `negativo`, `continuo` |
| **NO** | `calib_bN` (son solo fondo térmico para el confirmador IR) |

Lista las sesiones pendientes de etiquetar (todas menos `calib_*`):

```powershell
Get-ChildItem sessions -Directory | Where-Object { $_.Name -notmatch '_calib_' } | Select-Object -ExpandProperty Name
```

**Orden recomendado:** empieza por las de `fall` (son las más importantes y las
que tienen dos tramos), luego el resto.

---

## Paso 3 — La herramienta de etiquetado

Se lanza **una sesión cada vez**, pasando su carpeta:

```powershell
.\.venv\Scripts\python.exe tools\label_session_multimodal.py sessions\20260720_124816_guillermo_fall_centro\
```



Se abre una ventana con:
- **arriba**: `maxZ` / altura en el tiempo,
- **medio**: `vz` (velocidad vertical) / `az`,
- **abajo izq**: heatmap IR (MLX90640) del frame más cercano al cursor,
- **abajo dcha**: barra de color de la etiqueta actual.

### Controles

| Acción | Cómo |
|---|---|
| Seleccionar un rango temporal | **Arrastra** el ratón sobre el panel de arriba o el de en medio |
| Asignar etiqueta al rango | Pulsa la **tecla** de la etiqueta (tabla abajo) |
| Mover el heatmap IR en el tiempo | Mueve el ratón sobre las gráficas (scrub) |
| Deshacer la última asignación | `z` |
| **Guardar y salir** | `Enter` (escribe etiquetas en `radar.csv` **y** los `LabelSpan` del `manifest.json`) |
| Salir SIN guardar | `Esc` |

.\.venv\Scripts\python.exe tools\label_session_multimodal.py sessions\20260720_140552_guillermo_fall_dcha\
### Teclas de etiqueta

| Tecla | Etiqueta | Cuándo |
|---|---|---|
| `f` | `fall` | Desde el primer frame con `vz` claramente negativa (< −0.5) hasta ~2 s tras el impacto |
| `g` | `fall_lying` | Desde el fin de `fall` hasta que empieza a levantarse. **Debe cubrir ≥ 5 s** tras el impacto (el confirmador IR mira esa ventana) |
| `a` | `near_fall` | Tropiezo sin llegar a tocar el suelo |
| `s` | `sit` | Transición + permanencia sentado |
| `l` | `lie` | Tumbarse voluntario (≠ suelo tras caída) |
| `w` | `walk` | Marcha con desplazamiento neto |
| `t` | `stand` | De pie sin desplazarse |
| `n` | `none` | Habitación vacía / sin persona |

> **Backup automático:** la primera vez que guardas una sesión, la herramienta
> crea `radar.csv.bak`. Puedes reetiquetar sin miedo.
> `unknown` está **prohibido** en el dataset final: todo frame relevante debe
> quedar con una etiqueta.

---

## Paso 4 — Ejemplo completo (una sesión de caída)

Sesión de ejemplo: `20260720_124816_guillermo_fall_centro`. El guion grabado
fue: *de pie 5 s → caída → ≥ 8 s inmóvil en el suelo → levantarse.* Hay que
marcar **dos tramos**: la caída (`f`) y el suelo posterior (`g`).

**1) Abre la sesión:**

```powershell
.\.venv\Scripts\python.exe tools\label_session_multimodal.py sessions\20260720_124816_guillermo_fall_centro\
```

**2) Identifica la caída en la gráfica.** En el panel de `vz` verás un valle
marcado (velocidad vertical muy negativa, típicamente `vz ≤ −1.15 m/s`). Ese
pico hacia abajo es el instante de la caída. En el panel de `maxZ` verás que la
altura cae de "de pie" (~1.2–1.7 relativo) a "suelo" (~0.3–0.5).

**3) Marca `fall`.** Arrastra sobre la gráfica **desde el primer frame donde
`vz` empieza a bajar claramente** hasta **~2 s después del impacto** (cuando la
altura ya está en el suelo y `vz` vuelve a ~0). Con el rango seleccionado, pulsa
**`f`**. El tramo se pinta de rojo.

**4) Marca `fall_lying`.** Arrastra **desde el final del tramo anterior** (justo
donde el cuerpo ya está en el suelo) hasta **justo antes de que empiece a
levantarse** (cuando `maxZ` vuelve a subir). Asegúrate de que este tramo dure
**≥ 5 s**. Con el rango seleccionado, pulsa **`g`**. Se pinta de rojo oscuro.

```
 vz │        de pie          suelo (inmóvil)      se levanta
    │ ───────────────╮                          ╭──────────
    │                 ╲   valle = caída         ╱
    │                  ╰─────╮          ╭──────╯
    │                        ╰──────────╯
    └──────────────────────────────────────────────────────► t
                    [====f====][========g========]
                     caída        fall_lying (≥5 s)
```

**5) Revisa y guarda.** Si te equivocaste, `z` deshace el último tramo. Cuando
los dos tramos estén bien, pulsa **`Enter`** para guardar. La ventana se cierra.

**6) Siguiente sesión.** Repite el comando con el siguiente `id`.

### Cómo etiquetar el resto de actividades

| Actividad | Tramos a marcar |
|---|---|
| `nearfall_*` | Un tramo `a` sobre el tropiezo (no hay suelo, no lleva `g`) |
| `sit_*` | Un tramo `s` que cubra la bajada + permanencia sentado |
| `lie_*` | Un tramo `l` que cubra el tumbarse + permanencia tumbado |
| `walk_*` | Un tramo `w` sobre toda la marcha |
| `stand_*` | Un tramo `t` sobre el rato de pie |
| `none_*` | Un tramo `n` sobre toda la sesión (sala vacía) |
| `negativo_*` | **Varios** tramos: etiqueta cada segmento por lo que hacías (walk, stand, sit, none…). No debe quedar ningún `fall` |
| `continuo_*` | **Varios** tramos siguiendo la secuencia: `w` al caminar, `t`/`s` al pararte/sentarte, y en la caída marca `f`+`g` igual que en el ejemplo |

---

## Paso 5 — Sanity check tras etiquetar

Cuando termines **todas** las sesiones, verifica que al menos una de caída
produce ventanas `fall`:

```powershell
.\.venv\Scripts\python.exe tools\feature_engineering.py sessions\20260720_124816_guillermo_fall_centro\
```

Debe extraer **≥ 1 ventana `fall`**. Si una sesión de caída no produce ninguna,
revisa su etiquetado (probablemente el tramo `f` quedó demasiado corto o mal
situado).

---

## Paso 6 — Siguiente fase

Con todo etiquetado, pasas a la **Fase 7 — Entrenar y evaluar** (ver
`07-dia-de-captura.md`, sección Fase 7). Recuerda que lo primero de esa fase es
mover los `calib_*` fuera de `sessions\`:

```powershell
mkdir sessions_calib
move sessions\*_calib_* sessions_calib\
.\.venv\Scripts\python.exe tools\train_model.py --data sessions\*\ --no-dl
```

---

## Recordatorios

- Usa **siempre** `.\.venv\Scripts\python.exe`, nunca `python` a secas.
- Las `fall` y las `continuo` llevan **dos tipos de tramo** (`f` + `g`); no
  olvides el `g` (fall_lying) o el confirmador IR no tendrá ventana que mirar.
- El tramo `g` debe durar **≥ 5 s**.
- Guarda con `Enter` en cada sesión antes de pasar a la siguiente.
- Las `calib_bN` **no se etiquetan**.
- Criterios completos y casos dudosos: `docs/labeling_protocol.md`.

## Limitación conocida — punto ciego IR (dataset 2026-07-20/21)

En un lado de la sala (izquierda física ≈ lado ancho/+X del radar) la cámara IR
**no capta a la persona**. Consecuencias al etiquetar y analizar:

- **Etiquetar caídas de ese lado**: hazlo mirando la señal `vz`/`maxZ` del
  **radar**; el heatmap IR estará vacío pero no lo necesitas para etiquetar.
- **Fusión IR (Fase 7.3)**: la Regla-0 del confirmador (`ir_confirmer.py`) veta
  como *ghost track* cualquier caída sin blob IR previo → caídas reales de ese
  lado salen `vetoed` (fail-closed). **El radar-only no se ve afectado.** Esto es
  material del análisis fail-closed del TFM, no un fallo.
- **Analiza por coordenada `x` real, no por el nombre**: algunas sesiones de
  caída del 2026-07-20 tienen `izq`/`dcha` intercambiados (confirmado por la
  posición X), así que el nombre no es fiable para separar por lado.
- Sesión descartada por señal de radar mala: `20260720_140552_guillermo_fall_dcha`
  (movida a `sessions_excluidas/`).
