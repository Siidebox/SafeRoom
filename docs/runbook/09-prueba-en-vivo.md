# Prueba en vivo — detector ML + dashboard

Guía para validar el sistema completo end-to-end en la Raspberry Pi:
**radar → `radar_reader.py --ml-model` → detección → `dashboard`** (alerta en
navegador + Telegram opcional).

Es la prueba de despliegue de la Fase 7: comprobar que el modelo entrenado
dispara alertas reales en tiempo real, no solo en replay offline.

Asume que estás en la Pi (usuario `guillermo`), repo en `~/SafeRoom`.

---

## 0. Requisitos previos

- [ ] Modelo entrenado presente: `~/SafeRoom/models/fall_detector_xgb.pkl`
      (si no, ver [05-train-eval.md](05-train-eval.md)).
- [ ] Radar IWR6843AOPEVM conectado y flasheado. Aparecen `/dev/ttyUSB0`
      (CLI) y `/dev/ttyUSB1` (datos).
- [ ] Los **dos venvs** de la Pi (no los mezcles):
      - `~/SafeRoom/.venv` → radar / ML (ejecuta `radar_reader.py`).
      - `~/saferoom_cam`   → dashboard server (tiene `fastapi`, `httpx`, `uvicorn`).
- [ ] **Pulsa RST** en el radar antes de empezar (se queda en mal estado entre
      sesiones).

---

## 1. Arrancar el dashboard (Terminal 1)

```bash
~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
```

Debe imprimir `SafeRoom dashboard ready — db=/home/guillermo/SafeRoom/dashboard.db`.
Deja la terminal abierta (`Ctrl+C` para parar).

Abre en el navegador:
- Desde la Pi: <http://localhost:8000/>
- Desde otro dispositivo: <http://piSafeRoom.local:8000/> o la IP Tailscale.

### Telegram (opcional)

Para recibir alertas en el móvil, exporta las variables **antes** de arrancar
el dashboard:

```bash
export SAFEROOM_TG_TOKEN=<token-del-bot>
export SAFEROOM_TG_CHAT_ID=<chat-id>
~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
```

Solo `fall_fast`, `fall_confirmed`, `fall_failopen` y `faint` disparan Telegram.
`fall_candidate` no (es log-only).

---

## 2. Arrancar el radar con modelo + dashboard (Terminal 2)

Pulsa **RST** en el radar, luego (desde la raíz del repo, para que resuelvan
las rutas relativas `models/` y `logs/`):

```bash
cd ~/SafeRoom
.venv/bin/python tools/radar_reader.py \
  --cli /dev/ttyUSB0 --data /dev/ttyUSB1 \
  --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
  --ml-model models/fall_detector_xgb.pkl \
  --dashboard http://localhost:8000
```

Al arrancar debe imprimir:

```
[dashboard] posting fall/faint events to http://localhost:8000
```

Si no aparece esa línea, el flag `--dashboard` no se pasó → revísalo.

### Variantes

- **Con visualizador** (añade ventana 2D):  añade `--plot`.
- **Con fusión IR** (confirmador MLX90640): añade `--plot --ir`. La fusión IR
  **solo funciona con `--plot`** (en modo headless el confirmer está desactivado
  por diseño). Recuerda `--ir-rotate 90` si el montaje lo requiere.
- **Sin dashboard** (offline, como antes): quita `--dashboard`. Sigue
  imprimiendo en consola y registrando en `logs/fall_events.jsonl`.

---

## 3. Provocar una caída y qué esperar

Provoca una caída controlada en la zona cubierta.

**En consola (Terminal 2):**
```
[ML] FALL DETECTED — Track 3 (p=0.98)
*** FALL DETECTED — Track 3 at (0.40, 2.10, 0.15) m ***
```

**En el dashboard (navegador):** aparece una alerta crítica y el evento en el
historial. Tipos que verás según la configuración:

| Evento | Cuándo | `source` | ¿Alerta crítica? |
|---|---|---|---|
| `fall_confirmed` | Detección del **modelo ML** | `radar_ml` | Sí |
| `fall_fast` | Caída por **reglas**, sin confirmer IR | `radar` | Sí |
| `fall_confirmed` | Caída por reglas, IR confirma (con `--ir`) | `radar` | Sí |
| `fall_failopen` | Caída por reglas, IR no disponible | `radar` | Sí |
| `fall_candidate` | Caída por reglas, IR **veta** (fail-closed) | `radar` | No (log-only) |
| `faint` | Inmovilidad en suelo (Tier-2) | `radar` | Sí |

> El modelo ML y las reglas corren **en paralelo e independientes**: una misma
> caída puede generar un `fall_confirmed` (ML, `source=radar_ml`) y un
> `fall_fast`/`fall_confirmed` (reglas, `source=radar`). Es lo esperado.

---

## 4. Verificar que funcionó

Con el dashboard aún corriendo, desde cualquier terminal:

```bash
# Últimos eventos registrados (JSON):
curl -s http://localhost:8000/events?limit=10 | python3 -m json.tool

# Estado actual (presencia, alerta activa, sensores online):
curl -s http://localhost:8000/state | python3 -m json.tool
```

También quedan persistidos:
- Base de datos del dashboard: `~/SafeRoom/dashboard.db` (SQLite, sobrevive a
  reinicios).
- Log offline para tesis: `~/SafeRoom/logs/fall_events.jsonl` (una línea JSON
  por evento, con `event_type` y `payload`).

Para reconocer/limpiar la alerta activa en el dashboard usa el botón de la UI
(o `curl -X POST http://localhost:8000/ack_alert`).

---

## 5. Problemas frecuentes

| Síntoma | Causa / acción |
|---|---|
| No sale `[dashboard] posting ...` | Falta `--dashboard URL` en el comando. |
| Radar arranca pero el dashboard no recibe nada | ¿Dashboard arrancado en Terminal 1? Prueba `curl http://localhost:8000/state`. El notifier descarta eventos si el server está caído (no bloquea el radar). |
| `[WARN] Could not init dashboard notifier` | URL mal formada. Usa `http://localhost:8000` (sin barra final). |
| `Could not load ML model` | Ruta del `.pkl` incorrecta o ejecutaste fuera de `~/SafeRoom`. Corre desde la raíz del repo. |
| No aparecen `/dev/ttyUSB*` | Recablear USB, `dmesg | tail`, pulsar RST. |
| `Dropping frame` / fps bajos | RST del radar y reintenta. |
| La fusión IR no actúa en headless | Por diseño: usa `--plot --ir`. |
| Alerta no llega a Telegram | Exporta `SAFEROOM_TG_TOKEN` + `SAFEROOM_TG_CHAT_ID` **antes** de arrancar el dashboard; `fall_candidate` nunca dispara Telegram. |

---

## 6. Parar

- Terminal 2: `Ctrl+C` (cierra el reader; el notifier se apaga limpio).
- Terminal 1: `Ctrl+C` (para el dashboard).

El histórico persiste en `dashboard.db`. Para empezar de cero:
`rm ~/SafeRoom/dashboard.db` (la próxima ejecución crea una BD vacía).
