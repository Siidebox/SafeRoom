# Hardware and deployment

## Bill of materials

| Component | Role |
|-----------|------|
| TI IWR6843AOPEVM | 60 GHz FMCW radar with antenna-on-package. The only sensor required. |
| Raspberry Pi 5 (8 GB) | Edge host: parses TLVs, runs detection and alerting. |
| MLX90640 (optional) | 24x32 thermal array used only as a fall confirmer in experiments. |
| Wall mount | `hardware/case.scad`, printed enclosure. |

The project deliberately has no DCA1000 and no MMWAVEICBOOST: everything runs
from the EVM's own UART. Cameras are secondary and used only for
synchronization, labelling and comparison, never for the detection path.

## Mounting geometry

The deployment room is 5.10 x 3.7 m. The radar sits on a support case mounted
on a short wall, 1.30 m from the corner.

| Parameter | Value | Measured |
|-----------|-------|----------|
| Height above floor | 2.04 m | 2026-04-20 |
| Azimuth | 0 deg | |
| Downward tilt | 10 deg | 2026-04-20 |

These must match `sensorPosition` in the active `.cfg` and the
`--sensor-height` / `--sensor-tilt` arguments passed to `saferoom-read`.

## Flashing

1. Flash `code/People_Tracking/3D_People_Tracking/prebuilt_binaries/3D_people_track_6843_demo.bin`
   with TI UniFlash, or load it through a CCS debug session.
2. Connect the CLI UART at 115200 baud.
3. Send a `.cfg` line by line, ending with `sensorStart`
   (`saferoom-config` does this).
4. Read TLV frames from the data UART at 921600 baud.

Useful CLI commands: `sensorStop`, `flushCfg`, `sensorStart [0]`, `trackingCfg`,
`boundaryBox`, `staticBoundaryBox`, `presenceBoundaryBox`, `gatingParam`,
`stateParam`, `allocationParam`, `maxAcceleration`.

## Runtime configuration

`.cfg` files under `code/People_Tracking/3D_People_Tracking/chirp_configs/` are
replayed over the CLI UART at startup. They set:

- RF parameters: `profileCfg`, `chirpCfg`, `frameCfg`
- Detection thresholds: `dynamicRACfarCfg`, `staticRACfarCfg`
- Antenna geometry: `antGeometry0/1`, `antPhaseRot`
- Tracker: `trackingCfg`, `gatingParam`, `stateParam`, `allocationParam`
- Scene bounds: `staticBoundaryBox`, `boundaryBox`, `presenceBoundaryBox`
- Sensor pose: `sensorPosition`

TI's stock configurations are named by board variant and range
(`ISK_6m_default.cfg`, `AOP_9m_sensitive.cfg`, ...).

### The active SafeRoom configuration

`chirp_configs/SafeRoom_1p9m_4x6m.cfg`, derived from `AOP_6m_default.cfg`.
Every deviation from that baseline is annotated in the file's header comments;
the reasoning is:

| Parameter | Value | Why |
|-----------|-------|-----|
| `frameCfg` period | 50 ms (20 fps) | Validated on a Raspberry Pi 5 at 0 drops with 250 maxNumPoints |
| `trackingCfg` maxNumPoints | 250 | Safe once the firmware UART cap exists (see architecture.md) |
| `maxAcceleration` Z | 10.0 m/s^2 | Free fall is ~9.8 m/s^2; a lower cap makes the tracker lose the target mid-fall |
| `staticRACfarCfg` thresholds | 8.0 / 15.0 | Keeps static clutter from flooding the UART |
| `allocationParam` pointsThre | 20 | A fallen person reflects fewer points than a standing one |
| `boundaryBox` | -1.30 2.40 0.3 4.8 0.0 2.2 | Matches the room; the 1.30 m corner offset makes X asymmetric |
| `sensorPosition` | 2.04 m, azimuth 0, tilt 10 deg | Matches the mount above |

## Building the firmware

Code Composer Studio only; there is no command-line build. See
[`code/People_Tracking/3D_People_Tracking/README.md`](../code/People_Tracking/3D_People_Tracking/README.md).
