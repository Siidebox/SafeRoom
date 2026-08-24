# Architecture

SafeRoom detects presence, immobility and falls from a single 60 GHz FMCW radar.
The radar emits a sparse 3D point cloud, never an image, and every processing
stage runs either on the radar SoC or on a Raspberry Pi inside the home.

## Layers

| Layer | Where it runs | What it does |
|-------|---------------|--------------|
| 1. Radar firmware | TI IWR6843 SoC | Signal processing, detection, clustering, tracking |
| 2. Configuration | `.cfg` over UART CLI | Sets RF profile, thresholds, tracker gains, scene bounds |
| 3. Host / edge | Raspberry Pi 5 | TLV parsing, fall logic, logging, ML inference, alerting |

Layers 1 and 2 live under `code/People_Tracking/3D_People_Tracking/`.
Layer 3 is the `saferoom` Python package under `src/`.

## Dual-processor firmware

The IWR6843 splits work across two heterogeneous cores that talk over a
hardware mailbox.

**MSS — ARM Cortex-R4F** (`src/6843/mss/`)

Frame control, UART CLI handling, runtime configuration, IPC coordination,
tracker management, and serialization of the output TLVs.
Entry point: `mss_main.c`. CLI parsing: `pcount3D_cli.c`.

**DSS — TI C674x DSP** (`src/6843/dss/`)

Real-time radar signal processing: the detection pipeline, beamforming,
Doppler and CFAR stages. Entry point: `dss_main.c`; SYS/BIOS configuration in
`pcount3D_dss.cfg`. Algorithm modules live under `dss/modules/`
(`caponBF2D/`, `detection/`, `postProcessing/`, `utilities/`).

## Signal pipeline

```
ADC input
  → Range FFT (HWA)              rangeprochwa.c
  → Capon beamforming (DSP)      dss/modules/caponBF2D/
  → Doppler processing
  → CFAR detection               dss/modules/detection/CFAR/
  → DBSCAN clustering
  → 3D Kalman tracker            trackerproc_3d.c
  → Height extraction            height_detection.c
  → UART TLV output              mss_main.c
```

## Fall detection

Two rule-based tiers run in parallel on every tracked target
(`saferoom.radar.detector.FallDetector`), calibrated against labelled hardware
sessions:

| Tier | Trigger | Window | Purpose |
|------|---------|--------|---------|
| FAST | `vz <= -1.15 m/s` for 3 consecutive frames | 150 ms | The impact phase of a real fall |
| FAINT | `maxZ < 0.80 m` with height std `< 0.10 m` | 30 s sustained | On the floor and not moving |

A trained classifier (`saferoom.ml.inference.MlFallDetector`) can run alongside
the rules to compare predictions and cut false positives, and an optional
thermal confirmer (`saferoom.ir.confirmer`) can veto ghost tracks. See
[limitations.md](limitations.md) for what these tiers cannot distinguish.

## UART output protocol

### Connection

| Port | Baud | Direction | Windows | Linux / RPi |
|------|------|-----------|---------|-------------|
| CLI | 115200 | host → sensor | COM4 | `/dev/ttyUSB0` |
| Data | 921600 | sensor → host | COM3 | `/dev/ttyUSB1` |

### Frame structure

```
[Magic 8 B][Header 32 B][TLV ...][TLV]
```

Magic word: `02 01 04 03 06 05 08 07`.
Header format `'<Q8I'` (40 bytes including the magic):
magic, version, totalPacketLen, platform, frameNum, cpuCycles,
numDetectedObj, numTLVs, subFrameNum.

### TLV types

Defined in `code/People_Tracking/3D_People_Tracking/src/6843/mmwdemo_tlv.h`,
parsed in `saferoom.radar.tlv`.

| TLV | ID | Format | Description |
|-----|----|--------|-------------|
| Compressed point cloud | 1020 | 20 B unit header + N x 8 B | Spherical: i8 elev, i8 az, i16 doppler, u16 range, u16 snr |
| 3D track list | 1010 | N x 112 B, `'<I27f'` | TID, position, velocity, acceleration, 16-float covariance, g, confidence |
| Target index | 1011 | N x 1 B | Point-to-track mapping (255 = unassigned) |
| Target height | 1012 | N x 12 B, `'<I2f'` | TID, maxZ, minZ |
| Presence indication | 1021 | 4 B uint32 | 1 = person present |

`maxZ - minZ` is the vertical extent of the radar cluster on a person, typically
0.3 to 0.5 m. It is not the person's height; see
[limitations.md](limitations.md).

## UART bandwidth budget

At 921600 baud and a 50 ms frame period:

```
921600 bits/s / 10 bits per byte / 20 fps = ~4,600 bytes per frame
```

| Configuration | Payload | Verdict |
|---------------|---------|---------|
| 250 points x 8 B + headers | ~2.2 KB | ~50 % margin. Validated at 0 drops on a Raspberry Pi 5 |
| 750 points x 8 B + headers | ~7.1 KB | Over budget. Produces "Dropping frame" errors |

### The firmware point-cloud cap

`maxNumPoints` in `trackingCfg` bounds only the Kalman tracker's input. The
stock TI demo still serialized every CFAR detection, up to
`MAX_RESOLVED_OBJECTS_PER_FRAME` (750, hardcoded in the SDK), which overruns the
budget above.

`mss_main.c` clamps the serialized count to the configured `maxNumPoints`:

```c
uint32_t uartPointCount = outputFromDSP->pointCloudOut.object_count;
uint32_t maxUartPts = (uint32_t)gMmwMssMCB.trackerCfg.trackerDpuCfg
                          .staticCfg.gtrackModuleConfig.maxNumPoints;
if (uartPointCount > maxUartPts) uartPointCount = maxUartPts;
```

This makes the output deterministic without touching the SDK constant, and
allowed raising `maxNumPoints` to 250 in the deployment configuration.
