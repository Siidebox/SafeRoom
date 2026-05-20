# Etiquetar sesiones grabadas

## GUI multimodal (recomendado)

```bash
~/SafeRoom/.venv/bin/python ~/SafeRoom/tools/label_session_multimodal.py ~/SafeRoom/sessions/<id>/
```

Abre una ventana con radar (maxZ, vz, az) sincronizado con un heatmap IR
del frame más cercano al cursor.

**Necesita ventana Qt** — láncalo desde la terminal del escritorio de la
Pi, no por SSH.

## Controles

- **Arrastrar** el ratón sobre el panel superior para seleccionar un rango
  temporal.
- Luego pulsa la tecla de la etiqueta:

| Tecla | Etiqueta |
|---|---|
| `f` | `fall` |
| `a` | `near_fall` |
| `s` | `sit` |
| `l` | `lie` |
| `w` | `walk` |
| `t` | `stand` |
| `n` | `none` |

- `z` deshace la última asignación.
- `Enter` guarda y sale.
- `Esc` sale sin guardar.

## Modo radar-only (fallback)

Si la sesión no tiene `thermal.npz` o quieres etiquetar solo radar:

```bash
~/SafeRoom/.venv/bin/python ~/SafeRoom/tools/label_session.py ~/SafeRoom/sessions/<id>/radar.csv
```

Mismas teclas que la versión legacy: `f`/`n`/`s`/`w`.

## Criterios

Lee `docs/labeling_protocol.md`. Resumen rápido:

- **Inicio de `fall`**: primer frame con `vz` claramente negativa.
- **Final de `fall`**: extender al menos 5 s tras el impacto (el confirmer
  IR mira esa ventana).
- **`unknown`** queda prohibido en el dataset final.
- **Backup** automático en `radar.csv.bak` la primera vez que guardas.

## Sanity check post-etiquetado

```bash
~/SafeRoom/.venv/bin/python ~/SafeRoom/tools/feature_engineering.py ~/SafeRoom/sessions/<id>/
```

Imprime cuántas ventanas se extraen y cuántas son `fall` vs `normal`. Si
una sesión de caída no produce ninguna ventana `fall`, revisa el etiquetado.
