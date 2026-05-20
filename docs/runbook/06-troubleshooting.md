# Diagnóstico y mantenimiento

## El radar no manda frames (drops o 0 fps)

1. Reset físico (botón RST) y reintenta.
2. Comprueba puertos USB:
   ```bash
   ls -l /dev/serial/by-id/
   ```
   Deben aparecer dos enlaces a `ttyUSB0` y `ttyUSB1`. Si no, desconecta /
   reconecta el cable USB de la EVM.
3. Manda `sensorStop` a mano si algo quedó atascado:
   ```bash
   ~/SafeRoom/.venv/bin/python -c "import serial,time; s=serial.Serial('/dev/ttyUSB0',115200,timeout=1); s.write(b'sensorStop\n'); time.sleep(0.5); print(s.read(s.in_waiting).decode())"
   ```

## La IR no calibra ("hot pixels detected")

- Sal de la habitación.
- Espera 60 s. El confirmer reintenta solo cada 60 s.
- Si insiste, revisa qué hay caliente en el FOV: una bombilla incandescente,
  un portátil, sol directo sobre una pared.

## Tests del confirmer

```bash
cd ~/SafeRoom && ~/SafeRoom/.venv/bin/python -m pytest tests/ -v
```

Debe dar `19 passed`. Si falla algo, algo se rompió en alguna actualización.

## Actualizar repo en la Pi

```bash
cd ~/SafeRoom && git pull --ff-only
```

Si hay cambios locales sin commit que bloquean el pull:

```bash
cd ~/SafeRoom && git stash && git pull --ff-only && git stash pop
```

(O `git stash drop` si los cambios eran obsoletos.)

## Instalar dependencias nuevas

```bash
~/SafeRoom/.venv/bin/pip install -r ~/SafeRoom/requirements.txt
```

## Procesos huérfanos del dashboard

Si arrancaste el dashboard varias veces y quedó algo colgando:

```bash
pgrep -f dashboard_server.py     # lista PIDs
pkill -f dashboard_server.py     # mátalos todos
```

## Ver últimos eventos persistidos

```bash
~/saferoom_cam/bin/python -c "import sqlite3; c=sqlite3.connect('/home/guillermo/SafeRoom/dashboard.db'); [print(r) for r in c.execute('SELECT ts,type,track_id,substr(details,1,80) FROM events ORDER BY id DESC LIMIT 10')]"
```

## Ver últimas decisiones del IR confirmer

```bash
tail -5 ~/SafeRoom/logs/fall_events.jsonl
```

Cada línea es un JSON con `event_type`, `payload.ir_decision`,
`payload.ir_features`, `payload.ir_rules`. Útil para depurar por qué una
caída se clasificó como `confirmed` vs `candidate`.

## Acceso por SSH desde tu PC

```bash
ssh guillermo@100.110.129.108
```

(Contraseña local — no se guarda en repo.)

(Hostname `piSafeRoom` no resuelve fuera de la LAN local; usa la IP
Tailscale.)
