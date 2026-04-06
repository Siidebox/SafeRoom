---
name: checkpoint
description: Use when the user wants to save a version/checkpoint of the current SafeRoom code and config, or when a working algorithm version has been found. Manages git commits, tags, and optional GitHub push.
---

# SafeRoom Checkpoint

Announce: "Using checkpoint skill to save current state."

## Context

This project is at `C:\Users\sideb\chicago\IIT\tfm\SafeRoom`.
GitHub remote: `https://github.com/Siidebox/SafeRoom.git`
Key files to track:
- `tools/radar_reader.py` — fall/faint detector + visualizer
- `code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg` — active radar config
- `CLAUDE.md`

## Step 1 — Check git status

```bash
cd "C:/Users/sideb/chicago/IIT/tfm/SafeRoom"
git status 2>&1
```

If git is not initialized:
```bash
git init
git remote add origin https://github.com/Siidebox/SafeRoom.git
git fetch origin 2>/dev/null || echo "new repo or no network"
```

## Step 2 — Show diff summary

```bash
git diff --stat HEAD 2>/dev/null || git diff --stat
```

Show the user a brief summary of what changed. Focus on:
- Which thresholds changed in FallDetector
- Which config parameters changed in the .cfg

## Step 3 — Ask for checkpoint description

Ask the user: **"Describe este checkpoint en una frase (ej: 'Tier1 sin maxZ ceiling, FAST_PERSIST=5')"**

Wait for the response before continuing.

## Step 4 — Commit

Stage the key files and commit:
```bash
git add tools/radar_reader.py
git add "code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg"
git add CLAUDE.md
git add -u  # any other tracked files that changed
git commit -m "<description from user>

SafeRoom checkpoint — $(date +%Y-%m-%d)
Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>"
```

## Step 5 — Tag (optional)

Ask: **"¿Quieres etiquetar este checkpoint? (ej: v0.3-fall-tier1-fixed)"**

If yes:
```bash
git tag <tag-name>
```

## Step 6 — Push (optional)

Ask: **"¿Subir a GitHub ahora?"**

If yes:
```bash
git push origin main --tags 2>/dev/null || git push origin master --tags
```

If push fails (no upstream), run:
```bash
git push -u origin main --tags 2>/dev/null || git push -u origin master --tags
```

## Checkpoint summary

After completing, show:

```
✓ Checkpoint guardado
  Commit: <hash>
  Tag: <tag or 'none'>
  GitHub: <pushed / not pushed>

Thresholds activos:
  FAST_VZ_THRESHOLD = <value>
  FAST_PERSIST      = <value>
  SLOW_PERSIST      = <value>
  FAINT_PERSIST     = <value>
  FAINT_STABILITY   = <value>
```

Extract the threshold values from `tools/radar_reader.py` using grep before showing.

## Common cases

- **"guarda el estado actual"** → run full flow
- **"checkpoint sin push"** → skip Step 6
- **"etiqueta la version que funcionaba"** → tag the last working commit (ask which one)
- **"muestra el historial"** → `git log --oneline -20`
- **"vuelve a la versión anterior"** → `git log --oneline -10`, ask which commit, then `git checkout <hash> -- tools/radar_reader.py`
