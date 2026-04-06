# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Working Rules

1. Think before acting. Read existing files before writing code.
2. Be concise in output but thorough in reasoning.
3. Prefer editing over rewriting whole files.
4. Do not re-read files you have already read unless the file may have changed.
5. Test your code before declaring done.
6. No sycophantic openers or closing fluff.
7. Keep solutions simple and direct.
8. User instructions always override this file.

## Project Overview

**SafeRoom** is a Master's Final Project focused on building a **privacy-preserving indoor monitoring system** based on **mmWave radar** for:

- presence detection,
- immobility / lack-of-motion detection,
- fall detection.

The broader system goal is elderly-care-oriented indoor monitoring with **local or edge processing** and minimal invasiveness.

This repository is specifically centered on the **TI 3D People Tracking firmware stack** for the **TI IWR6843AOPEVM mmWave radar**, which serves as the radar-side processing foundation of the project.

### Available hardware in the broader project

- **TI IWR6843AOPEVM**
- **Raspberry Pi 5 (8 GB RAM)**
- Possible future support sensors:
  - RGB camera
  - IR camera

### Hardware constraints

- No **DCA1000**
- No **MMWAVEICBOOST**
- Radar is the primary sensing technology
- Any camera is secondary and only for synchronization, labeling, comparison, or experiments

### Role of this repository

This repository is not the whole SafeRoom system. Its main purpose is to support and adapt the radar firmware and configuration layer used by the project.

When working in this repo, prioritize understanding and modifying:

- radar configuration behavior,
- people tracking behavior,
- detection outputs,
- firmware-side processing assumptions,
- UART CLI configuration flow,
- output data relevant for presence, immobility, and fall-related experiments.

If host-side software, Raspberry Pi scripts, logging pipelines, or dataset tools are added later, treat them as higher-level layers built on top of this firmware foundation.

## Build System

This is a **TI Code Composer Studio (CCS)** embedded project — there is no npm/make/pip-based primary build flow. Builds are done through the CCS IDE.

- **CCS workspace**: `C:\Users\sideb\workspace_ccstheia`
- **Main project specs**:  
  `code/People_Tracking/3D_People_Tracking/src/6843/3D_people_track_6843_mss.projectspec`  
  `code/People_Tracking/3D_People_Tracking/src/6843/3D_people_track_6843_dss.projectspec`
- **Compiler**: TI C/C++ Compiler v16.9.6.LTS
- **SDK dependency**: mmWave SDK 3.5.0.4, SYS/BIOS 6.73.01.01
- **Post-build**: Generates `.bin` using `out2rprc.exe` → `MulticoreImageGen.exe` → `crc_multicore_image.exe`
- **Prebuilt binary**:  
  `code/People_Tracking/3D_People_Tracking/prebuilt_binaries/3D_people_track_6843_demo.bin`

Runtime configuration is loaded via UART CLI using `.cfg` files from `chirp_configs/`.

### Workspace vs. repo

The CCS workspace (`C:\Users\sideb\workspace_ccstheia`) is the active build location. The source files in this repo are the canonical version and are kept in sync with the workspace.

Custom files that live in `code/People_Tracking/3D_People_Tracking/src/6843/` (project root level, used by `${PROJECT_ROOT}` in the `.cproject`):

| File | Purpose |
|------|---------|
| `height_detection.c` | SafeRoom custom — height-based fall/posture detection |
| `trackerproc_3d.c` | SafeRoom custom — 3D Kalman tracker processing |
| `objdetrangehwa.c` | Modified TI Range HWA DPC |
| `rangeprochwa.c` | Modified TI range processing DPU |
| `mmwdemo_tlv.h` | TLV output format definitions |

DSS algorithm modules are under `src/6843/dss/modules/`: `caponBF2D/`, `detection/` (CFAR), `postProcessing/` (matrix ops), `utilities/`.

When making changes in the workspace, copy modified files back to the repo to keep it in sync. All standard TI files (mss_main.c, dss_main.c, pcount3D_cli.c, tracker_utils.c, etc.) are identical between workspace and repo.

### Build and validation expectations

When making changes:

- do not assume a desktop-style build system,
- do not introduce tooling that conflicts with CCS unless explicitly requested,
- prefer minimal changes compatible with the existing TI SDK structure,
- validate by checking compile consistency, config compatibility, and runtime logic assumptions,
- test code before declaring work complete.

## Architecture

### System Context

The full SafeRoom system can be understood as:

1. **Radar firmware layer**  
   Runs on the IWR6843AOPEVM and performs radar signal processing, detection, clustering, and tracking.

2. **Configuration layer**  
   `.cfg` files define radar runtime behavior through UART CLI commands.

3. **Host / edge layer**  
   `tools/radar_reader.py` (and future Raspberry Pi scripts) consumes radar outputs, logs experiments, visualizes data, and implements higher-level logic for presence, immobility, and fall detection.

This repository mainly covers layers **1** and **2**, with `tools/` as the beginning of layer **3**.

### Dual-Processor Model (IWR6843 SoC)

The firmware is split across two heterogeneous processors that communicate via Mailbox IPC:

**MSS (Master SubSystem) — ARM Cortex-R4F**
- Entry: `src/6843/mss/mss_main.c`
- CLI parsing: `src/6843/mss/pcount3D_cli.c`
- Tracking utilities: `src/6843/mss/tracker_utils.c`
- Responsibilities:
  - frame control,
  - UART CLI handling,
  - runtime configuration,
  - IPC coordination,
  - tracker management,
  - communication of processed outputs.

**DSS (Digital Signal SubSystem) — TI C674x DSP**
- Entry: `src/6843/dss/dss_main.c`
- SYS/BIOS config: `src/6843/dss/pcount3D_dss.cfg`
- Responsibilities:
  - real-time radar signal processing,
  - detection pipeline execution,
  - beamforming / Doppler / CFAR stages,
  - data preparation for higher-level tracking.

### Signal Processing Pipeline (DSS → MSS)

```text
ADC Input
  → Range FFT (HWA) [rangeprochwa.c]
  → Capon Beamforming (DSP) [caponbeamforming/]
  → Doppler Processing
  → CFAR Detection
  → DBSCAN Clustering
  → 3D Tracking / Kalman Filter [trackerproc_3d.c]
  → Height Detection [height_detection.c]
  → UART Output to host
```

### Demo Variants

| Variant | Target chip | Notes |
|---------|------------|-------|
| `3D_People_Tracking` | IWR6843 ISK/AOP/ODS | Primary demo, 3D (x,y,z) tracking |
| `3D_People_Tracking_Low_Power` | IWR6843 | Same algorithm, reduced power profile |
| `Overhead_3D_People_Tracking` | IWR6843 | Sensor mounted overhead |
| `IWRL6432_Capon2D_People_Tracking` | IWRL6432 | 2D Capon beamforming, single-core |
| `IWRL6844_People_Tracking` | IWRL6844 | FreeRTOS on C66x DSS side |
| `WiFi_Data_Transmission` | IWR6843 + CC3235SF | Radar + Wi-Fi bridge (pre-built only) |

## Chirp Configuration Files

`.cfg` files (in each variant's `chirp_configs/`) are sent over UART to the sensor at startup. They configure:
- RF parameters (`profileCfg`, `chirpCfg`, `frameCfg`)
- Detection thresholds (`dynamicRACfarCfg`, `staticRACfarCfg`)
- Antenna geometry (`antGeometry0/1`, `antPhaseRot`)
- Tracker parameters (`trackingCfg`, `gatingParam`, `stateParam`, `allocationParam`)
- Scene boundaries (`staticBoundaryBox`, `boundaryBox`, `presenceBoundaryBox`)
- Sensor position (`sensorPosition`)

Config files are named by board variant and range: e.g., `ISK_6m_default.cfg`, `AOP_9m_sensitive.cfg`.

### Active SafeRoom config

`chirp_configs/SafeRoom_1p9m_4x6m.cfg` — tuned for the actual deployment scenario:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `frameCfg` period | 150 ms (6.7 fps) | UART bandwidth budget |
| `trackingCfg` maxNumPoints | 100 | Cap UART output (see firmware note below) |
| `maxAcceleration` Z | 10.0 m/s² | Required to track free-fall (~9.8 m/s²) |
| `staticRACfarCfg` thresholds | 15.0 / 25.0 | Prevent static clutter from flooding UART |
| `allocationParam` pointsThre | 15 | Fallen person generates fewer points |
| `boundaryBox` | -1.25 1.25 0.3 4.2 0.0 2.2 | Matches 4.5m×2.5m room |
| `sensorPosition` | height=1.75m, azimuth=0, tilt=15° | Tripod in corner |

Comments at the top of `SafeRoom_1p9m_4x6m.cfg` document every change from the `AOP_6m_default.cfg` baseline.

## UART Output Protocol

### Connection

- **CLI port**: COM4 at 115200 baud — send config commands
- **Data port**: COM3 at 921600 baud — receive TLV frames
- On Linux/RPi: typically `/dev/ttyUSB0` (CLI) and `/dev/ttyUSB1` (data)

### Frame structure

```
[Magic 8B][Header 32B][TLV...TLV]
```

- **Magic**: `02 01 04 03 06 05 08 07`
- **Header format**: `'<Q8I'` = 40 bytes total  
  Fields: magic(u64), version, totalPacketLen, platform, frameNum, cpuCycles, numDetectedObj, numTLVs, subFrameNum

### TLV types (defined in `mmwdemo_tlv.h`)

| TLV | ID | Format | Description |
|-----|----|--------|-------------|
| Compressed point cloud | 1020 | 20B units header + N×8B points | Spherical coordinates (i8 elev, i8 az, i16 doppler, u16 range, u16 snr) |
| 3D track list | 1010 | N×112B | `'<I27f'` — TID + pos + vel + acc + covariance(16f) + g + confidence |
| Target index | 1011 | N×1B | Point-to-track mapping (255 = unassigned) |
| Target height | 1012 | N×12B | `'<I2f'` — TID, maxZ, minZ |
| Presence indication | 1021 | 4B uint32 | 1 = person present |

**Height interpretation**: `maxZ - minZ` is the vertical extent of the radar cluster on a person (typically 0.3–0.5 m), not the person's absolute height. This relative value still works for fall detection.

### UART bandwidth budget (at 921600 baud, 150 ms frame period)

- Available: 13,824 bytes/frame
- With 100 points × 8B + headers ≈ 1.5 KB → comfortable
- Without the firmware cap: up to 750 points × 8B ≈ 7.1 KB → marginal, causes "Dropping frame" errors

### Firmware UART cap (SafeRoom modification)

`mss_main.c` line ~1735 — caps UART point cloud output to `maxNumPoints` (from `trackingCfg`).  
**Status: code applied, not yet compiled/flashed.** After flashing, `maxNumPoints` can safely be raised to 250.

```c
uint32_t uartPointCount = outputFromDSP->pointCloudOut.object_count;
uint32_t maxUartPts = (uint32_t)gMmwMssMCB.trackerCfg.trackerDpuCfg.staticCfg.gtrackModuleConfig.maxNumPoints;
if (uartPointCount > maxUartPts) uartPointCount = maxUartPts;
```

Why this was needed: `maxNumPoints` in `trackingCfg` only limits Kalman tracker input — the original firmware always serialized all CFAR-detected points (up to `MAX_RESOLVED_OBJECTS_PER_FRAME` = 750) regardless.

## Python Tools (`tools/`)

### `tools/radar_reader.py` — main host-side reader

Parses TLV frames directly from the data UART port. Replaces Industrial Visualizer for experiments.

```bash
# Send config + read with live plot:
python tools/radar_reader.py --cli COM4 --data COM3 \
  --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg \
  --plot --log session.csv

# Read only (config already sent):
python tools/radar_reader.py --cli COM4 --data COM3
```

Key classes:
- `FrameReader` — syncs to magic word, reads full frames, dispatches TLV parsers
- `FallDetector` — two methods: height-ratio over 2.5s window (same as Industrial Visualizer) + vz spike threshold
- `CsvLogger` — logs per-track data to CSV (timestamp, pos, vel, height, fall flag)
- `run_visualizer` — matplotlib top/side view in a background thread

`FallDetector` thresholds (tunable constants at top of class):
- `RATIO_THRESHOLD = 0.6` — fall if current height < 60% of 2.5s-ago height
- `VEL_THRESHOLD = -2.0` m/s — fall on strong downward velocity
- `COOLDOWN = 5.0` s — minimum gap between detections per track

**Status: created, not yet tested against live hardware.**

### `tools/send_config.py` — standalone config sender

```bash
python tools/send_config.py --port COM4 \
  --cfg code/People_Tracking/3D_People_Tracking/chirp_configs/SafeRoom_1p9m_4x6m.cfg
```

Used when you want to (re)send config without starting the full reader. `radar_reader.py --cfg` calls the same logic internally.

### Dependencies

```bash
pip install pyserial numpy matplotlib
```

## Flashing and Running

1. Flash `.bin` via **Uniflash** (TI tool) or load via CCS debug session
2. Connect UART (CLI port COM4) at 115200 baud
3. Send config file line-by-line, ending with `sensorStart`
4. Parse TLV output on the data port COM3 (921600 baud)

CLI commands: `sensorStop`, `flushCfg`, `sensorStart [0]`, `trackingCfg`, `boundaryBox`, `staticBoundaryBox`, `presenceBoundaryBox`, `gatingParam`, `stateParam`, `allocationParam`, `maxAcceleration`

## Key Documentation

All reference docs are under `code/People_Tracking/docs/IWR6843_People_Tracking_PDFs/`:
- `3D_people_tracking_demo_implementation_guide.pdf`
- `3D_people_tracking_detection_layer_tuning_guide.pdf`
- `3D_people_tracking_tracker_layer_tuning_guide.pdf`

Quick-access guides also in `guides/` at the repo root.

Per-variant HTML guides are in each variant's `docs/` folder (e.g., `3d_people_tracking_user_guide.html`).

## Known Limitations

- **Lateral movement blind spot**: Doppler measures radial velocity only. Motion perpendicular to the beam (e.g., walking across the room) produces near-zero Doppler and is harder to track. Not fixable in config — requires physical sensor reorientation.
- **Height values are relative**: `maxZ - minZ` from TLV 1012 reflects the vertical extent of the radar reflection cluster (~0.3–0.5 m), not the person's absolute height. Fall detection using this value works as a ratio, not an absolute threshold.
- **`MAX_RESOLVED_OBJECTS_PER_FRAME` = 750**: Hardcoded in the SDK. Changing it requires recompilation. The firmware UART cap works around this without changing the SDK constant.
