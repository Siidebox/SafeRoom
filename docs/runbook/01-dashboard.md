# Dashboard web

## Arrancar

```bash
~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
```

Mantén la terminal abierta. `Ctrl+C` para parar.

## Abrir en navegador

- Desde la propia Pi: <http://localhost:8000/>
- Desde otro dispositivo en la red: <http://piSafeRoom.local:8000/> o
  <http://100.110.129.108:8000/> (Tailscale).

## Telegram (opcional)

Si quieres alertas de caída por Telegram, exporta las variables ANTES de
arrancar el dashboard:

```bash
export SAFEROOM_TG_TOKEN=<token-del-bot>
export SAFEROOM_TG_CHAT_ID=<chat-id>
~/saferoom_cam/bin/python ~/SafeRoom/tools/dashboard_server.py
```

Solo `fall_confirmed`, `fall_failopen`, `fall_fast` y `faint` disparan
Telegram. `fall_candidate` no.

## Persistencia

Todos los eventos se guardan en `~/SafeRoom/dashboard.db` (SQLite). El
historial sobrevive a reinicios del dashboard.

## Borrar histórico (cuidado)

```bash
rm ~/SafeRoom/dashboard.db
```

La próxima ejecución crea una base de datos nueva vacía.
