# SafeRoom — Runbook

Comandos de operación rápida, agrupados por tarea. Todos asumen que estás en
la Raspberry Pi (usuario `guillermo`) y que el repo vive en `~/SafeRoom`.

| Tarea | Archivo |
|---|---|
| Lanzar el dashboard web | [01-dashboard.md](01-dashboard.md) |
| Live: radar + IR + confirmer (vista en pantalla) | [02-radar-live.md](02-radar-live.md) |
| Grabar sesiones para el dataset | [03-dataset-recording.md](03-dataset-recording.md) |
| Etiquetar sesiones grabadas | [04-labeling.md](04-labeling.md) |
| Entrenar y evaluar modelos | [05-train-eval.md](05-train-eval.md) |
| Diagnóstico y mantenimiento | [06-troubleshooting.md](06-troubleshooting.md) |
| **Día de captura del dataset (guía completa)** | [07-dia-de-captura.md](07-dia-de-captura.md) |

## Antes de cualquier prueba con radar

Pulsa el botón **RST** del IWR6843AOPEVM. El sensor a veces se queda en mal
estado entre sesiones; este reset lo deja limpio.

## Dos venvs distintos en la Pi

| Venv | Uso |
|---|---|
| `~/SafeRoom/.venv` | Todo lo que es radar / ML / confirmer / pytest |
| `~/saferoom_cam` | Solo el dashboard server (tiene `fastapi`, `httpx`) |

No los mezcles. Cada comando del runbook ya indica cuál usar.
