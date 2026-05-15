# SafeRoom enclosure redesign — bracket-mounted radar + IR camera + RPi 5

**Status**: approved
**Date**: 2026-05-14
**Author**: Guillermo Sánchez Recuero (with Claude)

## Goal

Rework `hardware/case.scad` so the IWR6843AOPEVM radar (already screwed to its
TI/Mistral L-bracket) and the MLX90640 IR camera share the upper compartment
of a single 3D-printable enclosure, with the Raspberry Pi 5 in the bottom
compartment. The 10° downward tilt that the radar needs for fall-detection is
created by a wedge **under the bracket**, not by tilting the bare PCB.

## Non-goals

- Compactness: generous margins everywhere, optimised for "fits comfortably
  and prints reliably", not for minimum volume.
- Modelling the TI heat sink: the user has it installed; its height is less
  than the debug-pin tails underneath, so the radar envelope already covers it.

## Inputs

- STEP-measured radar PCB: 115.27 × 25.27 × 1.60 mm.
- AOP chip centre offset from PCB centre: (-20.12, -0.20) mm.
- Bracket+radar envelope (user-provided, includes margin): **122 × 50 × 50 mm**
  (length includes 7 mm extra on the X+ side for debug pins/unused USB).
- Raspberry Pi 5 with Armor Lite V5 active cooler: 85 × 56 × ~22 mm.
- MLX90640 Waveshare module: 28 × 16 × 1.6 mm PCB, sensor offset 5.5 mm from
  PCB X centre.

## Architecture

Single rectangular case, three stacked compartments (bottom → top):

```
+---------------------------+
| RADAR compartment         |   bracket + radar on 10° wedge, USB exits side
+---------------------------+
| CAMERA compartment        |   MLX90640 on flat shelf (tilt parameterised,
+---------------------------+   defaults to 0°)
| RPi 5 compartment         |   all ports accessible through side walls
+---------------------------+
```

Top opens via a removable grilled lid (same pattern as the current design).
No internal floor between radar and camera compartments — they share airflow;
a single floor partition separates the RPi compartment from the sensor stack
so the I2C/UART dupont wires can pass through one small notch.

### Components

1. **RPi compartment**
   - 4 M2.5 standoffs raise the PCB; same pattern as current `case.scad`.
   - Cutouts in the walls for: USB-C (power), 2× micro-HDMI on -X face;
     4× USB-A and Ethernet on +X face.
   - Side ventilation slots already in current design carry over.

2. **Camera compartment**
   - Flat shelf with two M2 standoffs at the MLX90640 hole positions.
   - Optical aperture (Ø10 mm) on the -Y wall, centred on the sensor.
   - `cam_tilt_deg` parameter, default `0`. When non-zero, the shelf becomes
     a small wedge tilted by that angle about the X axis.

3. **Radar compartment**
   - Flat-topped 10° wedge sized 122 × 50 mm at the base (matches the
     bracket+radar paralelepiped that the user specified).
   - The wedge is solid except for the front face: the antenna aperture
     (20 × 20 mm) is cut through the -Y wall of the case at the height of
     the AOP chip on the tilted bracket.
   - The bracket attaches to the wedge with double-sided industrial tape;
     no fastener holes through the wedge.
   - **Cable exit slot**: 12 × 8 mm rectangular cutout in the +X wall of
     the radar compartment, at the height of the radar's micro-USB
     connector after the bracket is tilted. The micro-USB→USB-A cable
     exits here and plugs back into one of the RPi USB-A ports from
     outside the case.

### Internal partitions

- Floor between RPi and camera compartments: full plate with a 10 × 6 mm
  notch for the dupont cables (camera I2C/UART → RPi GPIO header).
- No partition between camera and radar compartments — open volume.

### Top lid

Same removable slotted grill as the current design, covering the whole top
face. Flange overlaps the case rim by `grill_overlap`. Friction fit with
`tol` tolerance.

## Dimensions (target)

Outer envelope estimate: ~145 × 80 × 130 mm. Final exact size derived from
the largest interior compartment plus margins (the radar compartment
typically drives the X dimension because of the 122 mm bracket length).

## Parameters (OpenSCAD)

New / changed config variables in `case.scad`:

```scad
// Bracket+radar envelope (user-provided, includes generous margin)
bracket_w       = 122;   // X
bracket_d       = 50;    // Y
bracket_h       = 50;    // Z (with bracket vertical face standing)

// Camera tilt — defer decision until tested
cam_tilt_deg    = 0;     // change to 10 if needed after first test

// Radar antenna aperture
aperture_w      = 20;
aperture_h      = 20;

// Radar USB cable exit slot
usb_slot_w      = 12;    // along Y
usb_slot_h      = 8;     // along Z
```

The bracket length 122 mm already includes the 7 mm margin the user wants on
the +X (debug) side, so the wedge sits centred under the bracket.

## Cable routing summary

- **Camera → RPi**: dupont jumpers through the floor notch (internal).
- **Radar → RPi**: USB cable exits the +X wall of the radar compartment,
  runs externally along the outside of the case, plugs into an RPi USB-A
  port (also exposed on the +X face of the RPi compartment).
- **Power**: USB-C cable into the -X wall of the RPi compartment.

## Verification plan

1. Render `case.scad` to STL with OpenSCAD CLI; confirm CGAL reports a simple
   manifold solid (no self-intersections).
2. Visually inspect the preview to confirm:
   - Wedge under bracket tilts toward -Y (front).
   - Antenna aperture is centred on AOP chip position after tilt.
   - USB cable slot aligns with radar micro-USB connector height.
   - All RPi port cutouts are at correct heights.
3. Optional: import STL into a slicer to confirm printability (overhangs,
   bridging, supports).

## Open questions (deferred)

- Final value of `cam_tilt_deg`: defer until user tests the MLX90640 FOV from
  ~2 m height. Decision later, no enclosure reprint needed if the answer is
  "keep at 0".
- Exact bracket hole pattern (if user later wants to switch from double-sided
  tape to screws): not modelled for now.
