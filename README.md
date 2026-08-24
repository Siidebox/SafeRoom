# SafeRoom — Privacy-Preserving Indoor Fall Detection with mmWave Radar

> Master's Final Project — Illinois Institute of Technology / Universidad Politécnica de Madrid
> Real-time presence, immobility and fall detection for elderly care, using a single 60 GHz radar sensor and edge processing. **No cameras, no images, no audio.**

[![CI](https://github.com/Siidebox/SafeRoom/actions/workflows/ci.yml/badge.svg)](https://github.com/Siidebox/SafeRoom/actions/workflows/ci.yml)
[![Hardware](https://img.shields.io/badge/Radar-TI%20IWR6843AOPEVM-red)](https://www.ti.com/tool/IWR6843AOPEVM)
[![Edge](https://img.shields.io/badge/Edge-Raspberry%20Pi%205-c51a4a)](https://www.raspberrypi.com/products/raspberry-pi-5/)
[![Firmware](https://img.shields.io/badge/Firmware-C%20%2F%20TI%20CCS-blue)]()
[![Host](https://img.shields.io/badge/Host-Python%203.10%2B-3776ab?logo=python&logoColor=white)]()
[![ML](https://img.shields.io/badge/ML-XGBoost%20%7C%20PyTorch-orange)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)](#license)

---

## Why this project

Falls are the leading cause of injury-related death in adults over 65. Most home-monitoring solutions rely on cameras, which raise privacy concerns, or on wearables, which are often not being worn at the critical moment.

**SafeRoom** uses a single 60 GHz FMCW radar mounted on the wall. The radar sees the *shape and motion* of people through a sparse 3D point cloud — **never an image** — and the entire detection pipeline runs on a Raspberry Pi inside the home. Nothing leaves the device.

---

## What this repo contains

A complete working stack: the radar firmware running on the SoC, the host-side TLV parser, the real-time visualizer, the dataset collection tools, and the ML pipeline used to train and deploy the fall classifier.

| Layer | Location | Stack |
|-------|----------|-------|
| **Radar firmware** | `code/People_Tracking/3D_People_Tracking/src/6843/` | C, TI CCS, mmWave SDK 3.5, SYS/BIOS, dual-core ARM + DSP |
| **Custom DSP / tracker** | `height_detection.c`, `trackerproc_3d.c`, `objdetrangehwa.c`, `rangeprochwa.c` | Modified TI DPC/DPU |
| **Radar configs** | `chirp_configs/*.cfg` | UART CLI |
| **Host TLV reader** | `saferoom.radar.tlv` | Python, pyserial, NumPy |
| **Real-time visualizer** | `saferoom.radar.viz` | PyQtGraph + OpenGL |
| **Rule-based fall detector** | `saferoom.radar.detector` | Two tiers: fast vz drop, sustained floor level |
| **ML pipeline** | `saferoom.ml.*` | XGBoost, scikit-learn, optional LSTM/CNN in PyTorch |
| **IR confirmer** | `saferoom.ir.confirmer` | MLX90640 thermal veto (experimental) |
| **Dashboard** | `saferoom.dashboard` | FastAPI + SSE, local-only |
| **Mechanical** | `hardware/case.scad` | OpenSCAD wall-mount case |

---

## System architecture

```
                ┌──────────────────────────────────────┐
                │ TI IWR6843AOPEVM (60 GHz FMCW radar) │
                │  ┌────────────┐    ┌──────────────┐  │
                │  │  ARM R4F   │◄──►│  C674x DSP   │  │
                │  │  (MSS)     │ I  │  (DSS)       │  │
                │  │  CLI / IPC │ P  │  Range FFT   │  │
                │  │  Tracker   │ C  │  Capon BF    │  │
                │  │  TLV TX    │    │  CFAR        │  │
                │  └─────┬──────┘    └──────────────┘  │
                └────────│─────────────────────────────┘
                         │ UART  (CLI 115200 / Data 921600)
                         ▼
                ┌──────────────────────────────────────┐
                │ Raspberry Pi 5  (edge host)          │
                │  • TLV parser                        │
                │  • Kalman-derived features           │
                │  • FallDetector (rules + ML)         │
                │  • Local CSV log + dashboard         │
                └──────────────────────────────────────┘
```

### Signal pipeline (on-chip)

```
ADC → Range FFT (HWA) → Capon Beamforming (DSP) → Doppler →
CFAR → DBSCAN cluster → 3D Kalman tracker → Height extraction → UART TLV
```

The firmware detects up to 750 CFAR points per frame, but the original TI demo serialized all of them regardless of the tracker's configured limit, overrunning the UART budget and dropping frames under load. A patch in `mss_main.c` caps the UART point cloud to the tracker's `maxNumPoints`, making the bandwidth deterministic — validated at 20 fps with 250 points and zero drops on a Raspberry Pi 5.

Full detail in [docs/architecture.md](docs/architecture.md).

---

## Fall detection logic

Two tiers run in parallel on each tracked target:

| Tier | Trigger | Window | Purpose |
|------|---------|--------|---------|
| **FAST** | `vz ≤ −1.15 m/s` for 3 consecutive frames | 150 ms | Catch the impact phase of a real fall |
| **FAINT** | `maxZ < 0.80 m` and height std `< 0.10 m` | 30 s sustained | Person on the floor and not moving — unconscious or unable to get up |

Both thresholds were tuned from labelled hardware sessions in the deployment room (5.10 × 3.7 m, sensor at 2.04 m, tilt 10°, offset 1.30 m from a short-wall corner).

A trained classifier (`MlFallDetector`) can run alongside the rule-based detector to compare predictions and reduce false positives. See [docs/ml_model_rationale.md](docs/ml_model_rationale.md) for why both exist.

---

## TLV output protocol

The radar emits binary frames over UART at 20 Hz:

```
[Magic 8B][Header 32B][TLV …]
```

| TLV | ID | Description | Size |
|-----|----|-------------|------|
| 1020 | Compressed point cloud (spherical, i8/i16 packed) | 8 B / point |
| 1010 | 3D track list (TID + pos + vel + acc + 16-float cov + g + conf) | 112 B / track |
| 1011 | Point → track index mapping | 1 B / point |
| 1012 | Per-track height (`maxZ`, `minZ`) | 12 B / track |
| 1021 | Presence indicator | 4 B |

Formats are defined in `code/People_Tracking/3D_People_Tracking/src/6843/mmwdemo_tlv.h` and parsed in `saferoom.radar.tlv`.

---

## Repository layout

```
SafeRoom/
├── code/People_Tracking/3D_People_Tracking/
│   ├── chirp_configs/                  # UART CLI configs sent on boot
│   │   └── SafeRoom_1p9m_4x6m.cfg      #   active tuned deployment config
│   ├── prebuilt_binaries/              # flashable firmware image (.bin)
│   └── src/6843/                       # firmware source (CCS project)
│       ├── mss/                        #   ARM Cortex-R4F  (control + tracker + TLV TX)
│       ├── dss/                        #   C674x DSP       (FFT / Capon / CFAR / DBSCAN)
│       ├── height_detection.c          #   SafeRoom custom
│       ├── trackerproc_3d.c            #   SafeRoom custom
│       └── *.c / *.h                   #   modified TI DPC/DPU
├── src/saferoom/                       # host-side Python package
│   ├── radar/                          #   TLV parsing, detector, CSV logging, visualizer
│   ├── ir/                             #   MLX90640 capture + fall confirmer
│   ├── dataset/                        #   session recording, manifests, labelling
│   ├── ml/                             #   features, training, evaluation, inference
│   ├── evaluation/                     #   event metrics + offline replay
│   └── dashboard/                      #   FastAPI monitoring UI
├── tests/                              # pytest suite
├── hardware/case.scad                  # OpenSCAD wall-mount enclosure
├── docs/                               # architecture, protocols, runbook
└── pyproject.toml
```

---

## Quick start

### 1. Install

```bash
pip install -e ".[viz,ml]"          # radar + visualizer + ML
pip install -e ".[dashboard]"       # add the monitoring dashboard
pip install -e ".[dev]"             # add pytest + ruff
```

### 2. Flash the firmware

Use **TI UniFlash** to flash:

```
code/People_Tracking/3D_People_Tracking/prebuilt_binaries/3D_people_track_6843_demo.bin
```

### 3. Connect

| Port (Windows) | Port (Linux / RPi) | Function | Baud |
|----------------|--------------------|----------|------|
| COM4 | `/dev/ttyUSB0` | CLI — send config | 115 200 |
| COM3 | `/dev/ttyUSB1` | Data — TLV frames | 921 600 |

### 4. Run

Live 2D + 3D visualization with the deployment config:

```bash
saferoom-read --cli COM4 --data COM3 \
  --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
  --plot --plot3d
```

Record a labelled dataset session (keys: `f` fall · `n` normal · `s` sitting · `w` walking):

```bash
saferoom-read --cli COM4 --data COM3 --cfg ... --ml-log --label-mode
```

Deploy a trained model:

```bash
saferoom-read --cli COM4 --data COM3 --ml-model models/fall_detector_xgb.pkl
```

---

## Command-line tools

| Command | Purpose |
|---------|---------|
| `saferoom-read` | Live acquisition, detection, visualization, logging |
| `saferoom-config` | Send a `.cfg` to the radar CLI port |
| `saferoom-record` | Record a synchronized radar + IR dataset session |
| `saferoom-label` / `saferoom-label-mm` | Post-hoc labelling, radar-only or multimodal |
| `saferoom-check-sync` | Validate radar–IR temporal alignment of a session |
| `saferoom-features` | Extract sliding-window features from labelled sessions |
| `saferoom-train` | LOSO-CV training (XGBoost, Random Forest, optional LSTM/CNN) |
| `saferoom-evaluate` | Confusion matrices, ROC, PR, feature importance, latency |
| `saferoom-replay` | Offline paired comparison: rules vs ML vs IR fusion |
| `saferoom-dashboard` | Local monitoring dashboard |

`--help` on any of them prints the full argument list.

---

## ML workflow

```
record  →  label  →  extract features  →  train (LOSO-CV)  →  evaluate  →  deploy
   │         │              │                   │                │            │
saferoom-  saferoom-    saferoom-          saferoom-       saferoom-   saferoom-read
 record     label-mm     features            train          evaluate    --ml-model
```

`saferoom-train` runs **Leave-One-Session-Out cross-validation**. On the current single-subject dataset this is *not* equivalent to leave-one-subject-out, and the reported metrics should be read accordingly — see [docs/limitations.md](docs/limitations.md).

`saferoom-evaluate` produces the figures used in the thesis: confusion matrix, ROC, PR curve, feature importance, and a per-frame inference latency histogram.

---

## Documentation

| | |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Firmware split, signal pipeline, TLV protocol, UART budget |
| [docs/hardware.md](docs/hardware.md) | Bill of materials, mounting geometry, flashing, `.cfg` reference |
| [docs/limitations.md](docs/limitations.md) | What this approach cannot do, and why |
| [docs/runbook/](docs/runbook/README.md) | Day-to-day operating commands |
| [docs/data_collection_protocol.md](docs/data_collection_protocol.md) | Capture protocol |
| [docs/labeling_protocol.md](docs/labeling_protocol.md) | Label vocabulary and criteria |

---

## Tests

```bash
pip install -e ".[dev,ml]"
pytest
ruff check .
```

53 tests cover the fall detector's clock handling, the IR confirmer's rules, event metrics, replay, and window labelling. CI runs both on every push.

---

## Known limitations

- **Lateral-motion blind spot** — Doppler measures only *radial* velocity. Motion perpendicular to the beam produces near-zero Doppler and is harder to track. Not fixable in config; it requires sensor reorientation.
- **Height is relative** — TLV 1012 reports the vertical extent of the radar reflection cluster (typically 0.3–0.5 m), not the person's true height. It still works as a floor-proximity signal.
- **`MAX_RESOLVED_OBJECTS_PER_FRAME = 750`** is hard-coded in the TI SDK. The firmware UART cap works around it without recompiling the SDK.
- **Voluntary floor sitting** cannot be distinguished from a slow collapse by radar alone. It is not handled in the elderly-care target use case.
- **IR fusion fails closed** on one side of the room, where the thermal camera has no view. It cuts false alarms but can suppress real falls there.
- **Single-subject dataset** — cross-validation is session-level, so subject independence is not demonstrated.

Each of these is explained in full in [docs/limitations.md](docs/limitations.md).

---

## Hardware

| Device | Role |
|--------|------|
| TI **IWR6843AOPEVM** (60 GHz, antenna-on-package) | Primary sensor |
| **Raspberry Pi 5** (8 GB) | Edge processing host |
| USB CP2105 dual UART bridge | Host ↔ radar link |
| MLX90640 thermal array (optional) | Experimental fall confirmer |
| 3D-printed case (`hardware/case.scad`) | Wall mount |

No cameras, no microphones, no cloud.

---

## License

MIT — see [`LICENSE`](LICENSE).

Third-party firmware components are derived from the [TI mmWave Industrial Toolbox](https://www.ti.com/tool/MMWAVE-INDUSTRIAL-TOOLBOX) and remain under their original TI license.

---

## Author

**Guillermo Sánchez Recuero** — Master's Final Project, Illinois Institute of Technology and Universidad Politécnica de Madrid.
