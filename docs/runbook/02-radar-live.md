# Live: radar + IR + confirmer

Vista en pantalla con los paneles 2D, 3D, IR y contador de caídas. El
confirmer corre en el mismo proceso y emite `fall_confirmed` /
`fall_failopen` / `fall_candidate` al dashboard.

## Requisitos previos

1. **Reset físico** del radar (botón RST en la EVM).
2. (Opcional) Dashboard arrancado en otra terminal — ver
   [01-dashboard.md](01-dashboard.md). Sin él el sistema sigue funcionando
   pero los eventos solo van a `logs/fall_events.jsonl`.
3. La habitación **vacía** los primeros 30 segundos (auto-calibración del IR).

## Comando completo

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/radar_reader.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --plot --plot3d --ir --ir-hz 16 --ir-rotate 90
```

**Tiene que ejecutarse desde la terminal del escritorio de la Pi** (no por
SSH), porque abre una ventana Qt.

## Flags útiles

| Flag | Para qué |
|---|---|
| `--plot` | Vista 2D (X-Y arriba, X-Z lateral) |
| `--plot3d` | Añade el panel 3D OpenGL |
| `--ir` | Panel IR + confirmer activo |
| `--ir-hz 16` | Tasa de refresh del MLX90640 (8 fps efectivos) |
| `--ir-rotate {0,90,180,270}` | Compensa la cámara torcida |
| `--no-confirmer` | Desactiva el confirmer (solo `fall_fast` legacy). Útil para evaluación radar-only |
| `--ml-model models/fall_detector_xgb.pkl` | Carga el modelo ML alongside el rule-based |

## Qué esperar al arrancar

```
[IR] live panel active @ 16 Hz, rotation 90°
[IR] confirmer enabled — calibrating background...
```

A los ~30 s la calibración termina. Si la habitación no estaba vacía:

```
[IR] calibration aborted — hot pixels detected
```

Sal de la habitación y espera 60 s — reintenta solo. Si insistes en entrar
demasiado pronto, no calibra nunca; el confirmer se queda en `failopen` y
toda caída tier-1 sale como `fall_failopen`.

## Qué ves cuando hay una caída

- Consola: `*** FALL DETECTED — Track N at (x, y, z) m ***`
- A los pocos ms el confirmer decide: `fall_confirmed`, `fall_failopen` o
  `fall_candidate`.
- Dashboard pinta la fila con su color (rojo, rojo+badge, naranja).
- `logs/fall_events.jsonl` recibe una línea con el payload completo
  (decisión, confidence, qué reglas pasaron).

## Inspeccionar `logs/fall_events.jsonl`

```bash
tail -5 ~/SafeRoom/logs/fall_events.jsonl
```

O con jq si lo tienes:

```bash
tail -1 ~/SafeRoom/logs/fall_events.jsonl | python -m json.tool
```
