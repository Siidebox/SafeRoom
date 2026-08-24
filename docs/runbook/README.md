# Runbook

Operational commands, grouped by task. They assume the package is installed
(`pip install -e ".[viz,ml,dashboard]"`) and that you are in the repository
root.

| Task | Guide |
|---|---|
| Start the web dashboard | [01-dashboard.md](01-dashboard.md) |
| Live view: radar + IR + confirmer | [02-radar-live.md](02-radar-live.md) |
| Record dataset sessions | [03-dataset-recording.md](03-dataset-recording.md) |
| Label recorded sessions | [04-labeling.md](04-labeling.md) |
| Train and evaluate models | [05-train-eval.md](05-train-eval.md) |
| End-to-end live deployment test | [06-live-test.md](06-live-test.md) |
| Diagnostics and maintenance | [07-troubleshooting.md](07-troubleshooting.md) |
| Telegram alerts setup | [08-telegram-alerts.md](08-telegram-alerts.md) |

Background reading: [architecture](../architecture.md),
[hardware and deployment](../hardware.md),
[data collection protocol](../data_collection_protocol.md),
[labelling protocol](../labeling_protocol.md).

## Before any test with the radar

Press the **RST** button on the IWR6843AOPEVM. The sensor sometimes ends up in
a bad state between sessions, and the reset clears it.

## Two environments on the Raspberry Pi

The dashboard needs FastAPI and httpx, the radar path does not. If you keep two
virtualenvs on the Pi, install the matching extras in each:

| Environment | Install | Used by |
|---|---|---|
| Radar / ML | `pip install -e ".[viz,ml]"` | `saferoom-read`, `saferoom-record`, training, tests |
| Dashboard | `pip install -e ".[dashboard]"` | `saferoom-dashboard` |

A single environment with `".[viz,ml,dashboard]"` works too and is simpler.
