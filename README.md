# SafeRoom — Privacy-Preserving Indoor Fall Detection with mmWave Radar

> Master's Final Project — Illinois Institute of Technology / UPM
> Real-time presence, immobility and fall detection for elderly care, using a single 60 GHz radar sensor and edge processing. **No cameras, no images, no audio.**

[![Hardware](https://img.shields.io/badge/Radar-TI%20IWR6843AOPEVM-red)](https://www.ti.com/tool/IWR6843AOPEVM)
[![Edge](https://img.shields.io/badge/Edge-Raspberry%20Pi%205-c51a4a)](https://www.raspberrypi.com/products/raspberry-pi-5/)
[![Firmware](https://img.shields.io/badge/Firmware-C%20%2F%20TI%20CCS-blue)]()
[![Host](https://img.shields.io/badge/Host-Python%203.10%2B-3776ab?logo=python&logoColor=white)]()
[![ML](https://img.shields.io/badge/ML-XGBoost%20%7C%20PyTorch-orange)]()
[![License](https://img.shields.io/badge/License-MIT-green.svg)](#license)

---

## Why this project

Falls are the leading cause of injury-related death in adults over 65. Most home-monitoring solutions rely on cameras (privacy concerns) or wearables (often not worn at the critical moment).

**SafeRoom** uses a single 60 GHz FMCW radar mounted on the wall. The radar sees the *shape and motion* of people through a sparse 3D point cloud — **never an image** — and runs the entire detection pipeline on a Raspberry Pi inside the home. Nothing leaves the device.

---

## What this repo contains

A complete, working stack — from the radar firmware running on the SoC, through the host-side TLV parser, real-time visualizer, dataset collection tools, and the ML pipeline used to train and deploy the fall classifier.

| Layer | Files | Stack |
|-------|-------|-------|
| **Radar firmware** | `code/People_Tracking/3D_People_Tracking/src/6843/` | C, TI CCS, mmWave SDK 3.5, SYS/BIOS, dual-core ARM + DSP |
| **Custom DSP / tracker** | `height_detection.c`, `trackerproc_3d.c`, `objdetrangehwa.c`, `rangeprochwa.c` | Modified TI DPC/DPU |
| **Radar configs** | `chirp_configs/*.cfg` | UART CLI |
| **Host TLV reader** | `tools/radar_reader.py` | Python, pyserial, NumPy |
| **Real-time visualizer** | `tools/radar_reader.py --plot --plot3d` | PyQtGraph + OpenGL |
| **Rule-based fall detector** | `FallDetector` in `radar_reader.py` | 2-tier: fast vz drop + sustained floor-level |
| **ML pipeline** | `tools/ml_*.py`, `feature_engineering.py`, `train_model.py`, `evaluate_model.py` | XGBoost, scikit-learn, optional LSTM/CNN in PyTorch |
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
                │  • radar_reader.py → TLV parser      │
                │  • Kalman-derived features           │
                │  • FallDetector (rule + ML)          │
                │  • Local CSV log                     │
                └──────────────────────────────────────┘
```

### Signal pipeline (on-chip)

```
ADC → Range FFT (HWA) → Capon Beamforming (DSP) → Doppler →
CFAR → DBSCAN cluster → 3D Kalman tracker → Height extraction → UART TLV
```

The firmware ships **750-point CFAR detections** but the original demo capped the UART output, dropping frames under load. A patch in `mss_main.c` caps the UART point cloud to the tracker's configured `maxNumPoints` so the bandwidth budget is deterministic — validated at 20 fps × 250 points with zero drops on the RPi 5.

---

## Fall detection logic

Two tiers run in parallel on each tracked target:

| Tier | Trigger | Window | Purpose |
|------|---------|--------|---------|
| **FAST** | `vz ≤ −1.15 m/s` for 3 consecutive frames | 150 ms | Catch the impact phase of a real fall |
| **FAINT** | `maxZ < 0.80 m` and height std `< 0.10 m` | 30 s sustained | Person on the floor and not moving — unconscious / unable to get up |

Both thresholds were tuned from labelled hardware sessions in the deployment room (4.5 × 2.5 m, sensor at 2.04 m, tilt 10°).

A trained ML classifier (`MlFallDetector`) can run alongside the rule-based detector to compare predictions and reduce false positives.

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

All formats documented in `code/People_Tracking/3D_People_Tracking/src/6843/mmwdemo_tlv.h`.

---

## Repository layout

```
SafeRoom/
├── code/People_Tracking/
│   └── 3D_People_Tracking/
│       ├── chirp_configs/              # UART CLI configs sent on boot
│       │   └── SafeRoom_1p9m_4x6m.cfg  #   active tuned deployment config
│       ├── prebuilt_binaries/          # flashable firmware image (.bin)
│       └── src/6843/                   # firmware source (CCS project)
│           ├── mss/                    #   ARM Cortex-R4F  (control + tracker + TLV TX)
│           ├── dss/                    #   C674x DSP       (FFT / Capon / CFAR / DBSCAN)
│           ├── height_detection.c      #   SafeRoom custom
│           ├── trackerproc_3d.c        #   SafeRoom custom
│           └── *.c / *.h               #   modified TI DPC/DPU
├── tools/                              # host-side Python
│   ├── radar_reader.py                 #   main: parse TLV + visualize + detect
│   ├── send_config.py                  #   standalone CLI config sender
│   ├── ml_logger.py                    #   extended CSV logger
│   ├── label_session.py                #   post-hoc interactive labeller
│   ├── feature_engineering.py          #   sliding-window features
│   ├── train_model.py                  #   LOSO-CV training
│   ├── evaluate_model.py               #   ROC / PR / confusion matrix
│   └── ml_inference.py                 #   MlFallDetector for deployment
├── scripts/basic_read.py               # minimal UART connectivity test
├── src/radar/                          # reusable radar helpers
├── hardware/case.scad                  # OpenSCAD wall-mount enclosure
├── CLAUDE.md                           # engineering notes / context
└── requirements.txt
```

---

## Quick start

### 1. Install dependencies

```bash
pip install pyserial numpy pyqtgraph PyQt5 PyOpenGL
pip install xgboost scikit-learn joblib matplotlib   # ML pipeline
pip install torch                                    # optional (LSTM / CNN)
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

Live 2D + 3D visualisation with the deployment config:

```bash
python tools/radar_reader.py --cli COM4 --data COM3 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --plot --plot3d
```

Record a labelled dataset session (keys: `f` fall · `n` normal · `s` sitting · `w` walking):

```bash
python tools/radar_reader.py --cli COM4 --data COM3 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --ml-log --label-mode
```

Deploy a trained model:

```bash
python tools/radar_reader.py --cli COM4 --data COM3 --ml-model models/xgb_fall.pkl
```

---

## ML workflow

```
record  →  label  →  extract features  →  train (LOSO-CV)  →  evaluate  →  deploy
   │         │              │                   │                │            │
radar_reader label_session  feature_eng     train_model    evaluate_model  --ml-model
 --ml-log     .py             .py                .py             .py
```

`train_model.py` runs **Leave-One-Subject-Out cross-validation** so per-subject behaviour cannot leak between train and test folds — important on small clinical-style datasets.

`evaluate_model.py` produces the figures used in the thesis: confusion matrix, ROC, PR curve, feature importance, and a per-frame inference latency histogram.

---

## CLI arguments — `radar_reader.py`

| Argument | Default | Description |
|----------|---------|-------------|
| `--cli PORT` | *(required)* | CLI serial port |
| `--data PORT` | *(required)* | Data serial port |
| `--cfg PATH` | — | `.cfg` file to send at startup |
| `--plot` | off | Live 2D PyQtGraph visualization |
| `--plot3d` | off | Add a 3D OpenGL panel |
| `--log PATH` | auto | CSV log path (defaults to `logs/session_*.csv`) |
| `--frame-period SEC` | 0.05 | Expected frame period |
| `--sensor-height M` | 2.04 | Sensor height above floor |
| `--sensor-tilt DEG` | 10 | Sensor downward tilt |
| `--z-offset M` | 0 | Z correction in metres (e.g. `-0.20`) |
| `--ml-log` | off | Extended CSV (Kalman accel, PC stats, label column) |
| `--label-mode` | off | Real-time keyboard labelling thread |
| `--ml-model PATH` | — | Trained model (`.pkl` / `.pt`) |

---

## Known limitations (and why)

- **Lateral-motion blind spot** — Doppler only measures *radial* velocity. Motion perpendicular to the beam produces near-zero Doppler and is harder to track. Not fixable in config; requires sensor reorientation.
- **Height is relative** — TLV 1012 reports the vertical extent of the radar reflection cluster (typically 0.3–0.5 m), not the person's true height. Still works as a ratio for fall detection.
- **`MAX_RESOLVED_OBJECTS_PER_FRAME = 750`** is hard-coded in the TI SDK. The firmware UART cap works around it without recompiling the SDK.
- **Voluntary floor sitting** cannot be distinguished from a slow collapse by radar alone. Not handled in the elderly-care target use case; would require a secondary sensor (e.g. IR camera).

---

## Hardware

| Device | Role |
|--------|------|
| TI **IWR6843AOPEVM** (60 GHz, antenna-on-package) | Primary sensor |
| **Raspberry Pi 5** (8 GB) | Edge processing host |
| USB CP2105 dual UART bridge | Host ↔ radar link |
| 3D-printed case (`hardware/case.scad`) | Wall mount |

No cameras, no microphones, no cloud.

---

## License

MIT — see [`LICENSE`](LICENSE).

Third-party firmware components are derived from the [TI mmWave Industrial Toolbox](https://www.ti.com/tool/MMWAVE-INDUSTRIAL-TOOLBOX) and remain under their original TI license.

---

## Author

**Guillermo Sánchez Recuero** — Master's Final Project, Illinois Institute of Technology + Universidad Politécnica de Madrid.
