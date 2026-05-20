# Grabar sesiones para el dataset

Comando puro de captura sincronizada radar + IR a una carpeta de sesión.
Sin GUI, sin confirmer — solo datos crudos para etiquetar después.

## Antes

1. Reset físico del radar (botón RST).
2. Habitación vacía los primeros 30 s (calibración IR).

## Sesión estándar (30 s)

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --duration 30 --ir-hz 16 --name <sujeto>_<actividad>_<posicion> --subject <sujeto> --notes "<orientacion>,<ropa>,<hora>"
```

Sustituye `<sujeto>`, `<actividad>`, `<posicion>`, etc. Ejemplo:

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --duration 30 --ir-hz 16 --name guillermo_fall_centro --subject guillermo --notes "frontal,camiseta_algodon,tarde"
```

## Sesión larga sin IR (solo radar)

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python tools/session_recorder.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --duration 60 --no-ir --name precheck_radar
```

## Verificar la sesión grabada

Después del comando el script imprime:

```
[REC] Radar: 600 frames, 0 drops, 20.04 fps
[REC] IR: 240 frames, 0 bad, 7.92 fps
[REC] Manifest written: sessions/<id>/manifest.json
```

Calidad mínima aceptable:
- `radar.fps_real >= 19.5`
- `thermal.fps_real >= 7.5`
- `radar.drops < 1% × frames`
- `thermal.bad_frames < 1% × frames`

Si no se cumple, **repite la sesión** (y revisa cableado / reset).

## Comprobar sincronía radar↔IR

```bash
~/SafeRoom/.venv/bin/python tools/check_sync.py sessions/<id> --max-drift-ms 50
```

Verás drift entre eventos de movimiento detectados en ambos streams.
Objetivo: `max |dt| <= 50 ms`.

## Listado y limpieza

```bash
ls -lh ~/SafeRoom/sessions/
rm -r ~/SafeRoom/sessions/<id>     # borrar una sesión concreta
```

Las sesiones están en `.gitignore`, no se suben a git por defecto.

## Protocolo completo del día

Lee `docs/data_collection_protocol.md` — clases, repartos objetivo (60 falls,
60 sits, etc.), variación obligatoria de posición/orientación/sujeto/ropa,
cronograma sugerido.
