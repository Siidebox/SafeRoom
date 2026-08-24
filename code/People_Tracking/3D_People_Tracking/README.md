# 3D People Tracking Firmware (IWR6843AOPEVM)

Radar-side firmware for SafeRoom. Runs on the TI IWR6843 SoC and emits a tracked
3D point cloud over UART.

## Upstream

Derived from the **3D People Tracking** demo in the
[TI mmWave Industrial Toolbox](https://dev.ti.com/tirex/explore/node?node=A__AJy3JCS6h-U0LNvzHwWCFA__radar_toolbox__1AslXXD__LATEST).

| Component | Version |
|-----------|---------|
| mmWave SDK | 3.5.0.4 |
| SYS/BIOS | 6.73.01.01 |
| TI C/C++ Compiler | v16.9.6.LTS |
| Target | IWR6843AOPEVM |

The vendor HTML user guides that shipped with the toolbox are not redistributed here;
download them from the TI Resource Explorer link above. The TI tuning PDFs used while
developing SafeRoom are kept locally under `code/People_Tracking/docs/` (not committed).

Only the `3D_People_Tracking` variant is included. The other toolbox demos
(Overhead, Low Power, WiFi, IWRL6432, IWRL6844) are not part of SafeRoom.

## SafeRoom modifications

| File | Change |
|------|--------|
| `src/6843/height_detection.c` | Height-based fall / posture detection (new) |
| `src/6843/trackerproc_3d.c` | 3D Kalman tracker processing |
| `src/6843/objdetrangehwa.c` | Modified TI Range HWA DPC |
| `src/6843/rangeprochwa.c` | Modified TI range processing DPU |
| `src/6843/mmwdemo_tlv.h` | TLV output format definitions |
| `src/6843/mss/mss_main.c` | UART point-cloud cap (see below) |

### UART point-cloud cap

`maxNumPoints` in `trackingCfg` only bounds the Kalman tracker input. The stock demo
still serialized every CFAR detection (up to `MAX_RESOLVED_OBJECTS_PER_FRAME` = 750),
overrunning the 921600-baud budget at 20 fps and producing "Dropping frame" errors.
`mss_main.c` now clamps the serialized point count to `maxNumPoints`, making the
bandwidth deterministic. See `docs/architecture.md` for the budget calculation.

## Building

Built through the TI Code Composer Studio IDE — there is no command-line build.

Import both project specs:

- `src/6843/3D_people_track_6843_mss.projectspec` (ARM Cortex-R4F)
- `src/6843/3D_people_track_6843_dss.projectspec` (C674x DSP)

The post-build step chains `out2rprc.exe` → `MulticoreImageGen.exe` →
`crc_multicore_image.exe` to produce the flashable image.

## Flashing

A prebuilt image is provided at
`prebuilt_binaries/3D_people_track_6843_demo.bin`. Flash it with **TI UniFlash**,
then send a runtime configuration from `chirp_configs/` over the CLI UART.

`chirp_configs/SafeRoom_1p9m_4x6m.cfg` is the active deployment configuration;
its header comments document every deviation from the `AOP_6m_default.cfg` baseline.
