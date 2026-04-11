# SafeRoom — Indoor Monitoring with mmWave Radar

> [!IMPORTANT]
> Work in progress — dataset collection and ML training pending.

Privacy-preserving indoor presence, immobility, and fall detection system using a millimeter-wave radar sensor. Designed for elderly-care scenarios with local/edge processing and no cameras required.

## Hardware

| Device | Role |
|--------|------|
| TI IWR6843AOPEVM (AOP variant) | Primary sensor |
| Raspberry Pi 5 (8 GB) | Edge processing host |
| USB CP2105 dual UART bridge | PC/RPi ↔ radar link |

## Repository Structure

```
SafeRoom/
├── code/People_Tracking/
│   └── 3D_People_Tracking/
│       ├── chirp_configs/          # .cfg files sent to radar over UART
│       │   └── SafeRoom_1p9m_4x6m.cfg  # active deployment config
│       ├── prebuilt_binaries/      # flashable .bin
│       └── src/6843/               # firmware source (CCS project)
├── tools/                          # host-side Python tools
│   ├── radar_reader.py             # main reader + visualizer + fall detector
│   ├── send_config.py              # standalone config sender
│   ├── ml_logger.py                # extended CSV logger for ML
│   ├── label_session.py            # post-hoc labeling UI
│   ├── feature_engineering.py      # sliding-window feature extraction
│   ├── train_model.py              # LOSO-CV training (XGBoost, RF, LSTM)
│   ├── evaluate_model.py           # thesis figures: ROC, confusion matrix…
│   └── ml_inference.py             # MlFallDetector for deployment
├── scripts/
│   └── basic_read.py               # minimal UART connectivity test
├── logs/                           # auto-saved CSV sessions (not tracked)
└── models/                         # trained model files (not tracked)
```

## Quick Start

### Dependencies

```bash
# Core
pip install pyserial numpy pyqtgraph PyQt5

# 3D visualization
pip install PyOpenGL

# ML pipeline
pip install xgboost scikit-learn joblib matplotlib

# Optional: deep learning models
pip install torch
```

### Serial ports

| Port (Windows) | Port (Linux/RPi) | Function | Baud |
|----------------|------------------|----------|------|
| COM4 | /dev/ttyUSB0 | CLI — send config commands | 115200 |
| COM3 | /dev/ttyUSB1 | Data — receive TLV frames | 921600 |

---

## Running the Radar Reader

`radar_reader.py` is the main tool. It sends the config, parses TLV output, visualizes tracks, and runs fall detection.

### Minimal — read only, no plot

```bash
python tools/radar_reader.py --cli COM4 --data COM3
```

### With config + live 2D plot

```bash
python tools/radar_reader.py --cli COM4 --data COM3 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --plot
```

### With 3D OpenGL panel

```bash
python tools/radar_reader.py --cli COM4 --data COM3 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --plot --plot3d
```

### With CSV log + ML labeling (dataset collection)

```bash
python tools/radar_reader.py --cli COM4 --data COM3 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --ml-log --label-mode
```
Keys during session: `f`=fall · `n`=normal · `s`=sitting · `w`=walking

### With trained ML model (deployment)

```bash
python tools/radar_reader.py --cli COM4 --data COM3 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg --ml-model models/xgb_fall.pkl
```

### All arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--cli PORT` | *(required)* | CLI serial port (e.g. `COM4` or `/dev/ttyUSB0`) |
| `--data PORT` | *(required)* | Data serial port (e.g. `COM3` or `/dev/ttyUSB1`) |
| `--cfg PATH` | — | `.cfg` file to send at startup |
| `--plot` | off | Live 2D PyQtGraph visualization |
| `--plot3d` | off | Add 3D OpenGL panel (requires PyOpenGL) |
| `--log PATH` | auto | CSV log path. Auto-saves to `logs/session_YYYYMMDD_HHMMSS.csv` if omitted |
| `--frame-period SEC` | 0.05 | Expected frame period in seconds |
| `--sensor-height M` | 2.05 | Sensor height above floor (metres) |
| `--sensor-tilt DEG` | 15.0 | Sensor downward tilt (degrees) |
| `--z-offset M` | 0 | Z correction in **metres** applied to all points/tracks. Negative = shift down (e.g. `--z-offset -0.20` = −20 cm) |
| `--ml-log` | off | Use extended ML CSV logger (adds ax/ay/az, point cloud stats, label column) |
| `--label-mode` | off | Real-time keyboard labeling thread (requires `--ml-log`) |
| `--ml-model PATH` | — | Trained model (`.pkl` or `.pt`). Runs alongside rule-based detector |

---

## Other Tools

### Send config only (without reading data)

```bash
python tools/send_config.py --port COM4 --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg
```

### Basic UART connectivity test

```bash
python scripts/basic_read.py
```

---

## ML Workflow

```
1. Record   python tools/radar_reader.py ... --ml-log --label-mode
2. Label    python tools/label_session.py logs/session_*.csv        (or use --label-mode above)
3. Extract  python tools/feature_engineering.py logs/
4. Train    python tools/train_model.py
5. Eval     python tools/evaluate_model.py
6. Deploy   python tools/radar_reader.py ... --ml-model models/xgb_fall.pkl
```

---

## Flashing the Firmware

1. Flash `code/People_Tracking/3D_People_Tracking/prebuilt_binaries/3D_people_track_6843_demo.bin` using **TI UniFlash**.
2. Connect UART (CLI port at 115200 baud).
3. Send config file line-by-line; end with `sensorStart`.
4. Parse TLV output on data port at 921600 baud.

---

## License

TBD
