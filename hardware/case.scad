// SafeRoom case — IWR6843AOPEVM + MLX90640 + Raspberry Pi 5
//
// Parametric enclosure. Edit the variables in the CONFIG section to match
// your physical measurements (nominal values from datasheets are used as
// placeholders). Render with F6 and export STL.
//
// Components:
//   - RADAR    : TI IWR6843AOPEVM, top shelf, tilted 10° downward
//   - IR CAM   : Waveshare MLX90640 thermal camera, just below the radar
//   - RPi 5    : with Armor Lite V5 active cooler, horizontal in the lower
//                compartment, fan exhausting upward through a removable grill
//
// Print orientation: case body upright as drawn (cooling grill side up).
// Material: PETG preferred (PLA acceptable with extra ventilation).

// ============================================================
// CONFIG — edit these to match your measurements
// ============================================================

// --- Wall / tolerance -----------------------------------
wall            = 2.4;     // outer wall thickness
tol             = 0.4;     // print tolerance for holes / snap-fits
$fn             = 48;

// --- Radar IWR6843AOPEVM (parsed from sprr418 Rev.G Altium ASCII) ---
// PCB is rectangular and elongated. The board carries both the Mission AOP
// section (with the 60 GHz radar) and the Breakaway debug section connected
// by a break-away line — we keep the board INTACT so the onboard USB-A works.
radar_pcb_w     = 115.0;   // X — total length (fab drawing: 115.01 mm)
radar_pcb_d     = 38.5;    // Y — width (fab drawing: 38.49 mm)
radar_pcb_t     = 1.6;
// 4 mounting holes — exact pitch parsed from Altium PcbDoc
radar_hole_dia  = 2.4;     // M2 clearance (drilled holes = 2.24 mm)
radar_hole_x    = 109.92;
radar_hole_y    = 34.61;
radar_tilt_deg  = 10;
// Mission USB (micro-USB Type-B, used for data + CLI) — confirmed from
// PDF swru546e fig 4-1: it exits through the SHORT X- edge of the PCB
// (the Mission AOP Board side). The other USB on the X+ end is power-only
// and we leave it INSIDE the case (no cutout) — RPi USB 3.0 powers the whole
// thing through this one.
radar_usb_h     = 3.0;     // micro-USB Type-B body height
radar_usb_w     = 8.0;     // micro-USB Type-B body width
radar_usb_edge  = "X-";    // Mission USB exits the X- short edge
radar_usb_z_off = 0.0;     // Z offset along the PCB face (centred)
// Heat sink (TI part, from PDF swru546e fig 4-5/4-6). Mounted on the BACK
// side of the EVM, over the AOP chip in the Mission Board. In our case the
// PCB is oriented antenna-down so the heat sink projects upward.
hs_w            = 17.2;    // X footprint
hs_d            = 30.0;    // Y footprint
hs_h            = 23.5;    // Z height (fin tips)
hs_clear        = 4.0;     // vertical clearance above the fins
// Heat sink centre relative to PCB centre — chip is on the Mission AOP
// Board (the LEFT half of the PCB). Estimated offset ~ -25 mm in X.
hs_offset_x     = -25.0;
hs_offset_y     = 0.0;     // roughly centred along the short axis
// The 60 GHz AOP antenna is integrated in the chip package on the TOP face.
// Keep the area facing forward (Y-) free — no cover/shelf in front of it.

// --- Camera MLX90640 Waveshare (measured 2026-05-12) ----
cam_pcb_w       = 28.0;    // X
cam_pcb_d       = 16.0;    // Y
cam_pcb_t       = 1.6;
cam_hole_dia    = 2.4;     // M2 clearance (2.0 + tol)
// Two mounting holes on the RIGHT side of the PCB (not centred).
// Coordinates relative to PCB centre:
cam_hole1_x     = 12.0;    // = 26 mm from left edge - 14 mm centre
cam_hole1_y     = -6.0;
cam_hole2_x     = 12.0;
cam_hole2_y     =  6.0;
cam_sensor_offset_x = 5.5; // sensor centre = 19.5 mm from left edge - 14 centre
cam_sensor_offset_y = 0.0; // sensor centred vertically (y = 8 mm of 16 mm)
cam_sensor_dia  = 8.0;     // optical aperture for the sensor element

// --- Raspberry Pi 5 + Armor Lite V5 ---------------------
rpi_pcb_w       = 85.0;
rpi_pcb_d       = 56.0;
rpi_pcb_t       = 1.6;
rpi_hole_dia    = 2.8;     // M2.5 clearance
rpi_hole_x_pitch = 58.0;   // X between hole centres
rpi_hole_y_pitch = 49.0;   // Y between hole centres
rpi_hole_inset  = 3.5;     // hole centre offset from nearest PCB corner
armor_height    = 16.0;    // = height of RPi USB-A connectors (Armor Lite V5 sits at the same level)
fan_dia         = 30.0;    // top fan diameter  <-- VERIFY
fan_offset_x    = 0.0;     // fan centre offset from RPi PCB centre, X
fan_offset_y    = 0.0;     // fan centre offset from RPi PCB centre, Y

// --- Port locations on RPi 5 (right-side long edge) ----
// Distances from the front-left corner of the PCB (looking from above,
// USB ports facing right). All nominal — verify if your PCB orients
// differently inside the case.
usb_a_y_start   = 9.0;     // first USB stack starts at Y = 9 mm
usb_a_y_pitch   = 18.0;    // spacing between USB stacks
usb_a_h         = 16.0;    // height of one USB-A stack
usb_a_w         = 13.5;    // width of one USB-A stack
eth_y           = 45.0;    // Ethernet RJ45 centre
eth_w           = 16.0;
eth_h           = 14.0;
// Short-edge ports (USB-C + 2x micro-HDMI)
usbc_x          = 11.0;    // USB-C distance from front edge
usbc_w          = 9.0;
usbc_h          = 4.0;
hdmi0_x         = 26.0;
hdmi1_x         = 39.5;
hdmi_w          = 7.5;
hdmi_h          = 4.0;

// --- Compartment sizing --------------------------------
// Inner clearances around components; tweak if needed.
rpi_clear_top    = 4.0;    // clearance above Armor Lite fan
rpi_clear_below  = 6.0;    // clearance below RPi PCB (standoffs)
rpi_clear_sides  = 5.0;    // clearance around RPi PCB in X/Y
radar_clear      = 4.0;    // clearance around radar PCB
cam_clear        = 4.0;    // clearance around camera PCB
shelf_gap        = 3.0;    // vertical gap between radar shelf and camera shelf

// --- Grill (removable top) -----------------------------
grill_thickness  = 2.4;
grill_bar_w      = 2.0;    // bar width
grill_slot_w     = 4.0;    // slot width between bars
grill_overlap    = 4.0;    // how far the grill flange overlaps the case rim

// ============================================================
// DERIVED DIMENSIONS — do not edit
// ============================================================

// Inner XYZ of the RPi compartment
rpi_inner_w = rpi_pcb_w + 2*rpi_clear_sides;          // X
rpi_inner_d = rpi_pcb_d + 2*rpi_clear_sides;          // Y
rpi_inner_h = rpi_clear_below + rpi_pcb_t + armor_height + rpi_clear_top;

// Inner XYZ of the radar shelf (allowing for tilt AND the heat sink)
radar_inner_w = max(radar_pcb_w + 2*radar_clear, rpi_inner_w);
radar_inner_d = radar_pcb_d + 2*radar_clear;
// Shelf must accommodate the wedge tilt, the PCB, the heat sink height,
// and a top clearance for fin airflow.
radar_shelf_h = radar_pcb_d * sin(radar_tilt_deg)
              + radar_pcb_t + hs_h + hs_clear + 2;

// Inner XYZ of the camera shelf
cam_inner_w = max(cam_pcb_w + 2*cam_clear, rpi_inner_w);
cam_inner_d = cam_pcb_d + 2*cam_clear;
cam_shelf_h = cam_pcb_t + cam_clear*2 + 2;

// Outer case size
case_w = max(radar_inner_w, cam_inner_w, rpi_inner_w) + 2*wall;
case_d = max(radar_inner_d, cam_inner_d, rpi_inner_d) + 2*wall;
case_h = wall + rpi_inner_h + shelf_gap + cam_shelf_h + shelf_gap + radar_shelf_h + wall;

// Z coordinates (origin at bottom outside face)
z_rpi_floor   = wall + rpi_clear_below;          // top of RPi standoffs
z_rpi_top     = z_rpi_floor + rpi_pcb_t + armor_height;
z_grill       = z_rpi_top + rpi_clear_top;       // grill sits here
z_cam_floor   = z_grill + shelf_gap;
z_radar_floor = z_cam_floor + cam_shelf_h + shelf_gap;

echo("Case outer:", case_w, "x", case_d, "x", case_h, "mm");

// ============================================================
// MODULES
// ============================================================

module rounded_box(size, r=2) {
    minkowski() {
        cube([size[0]-2*r, size[1]-2*r, size[2]-r*0.01]);
        sphere(r=r);
    }
}

// 4 cylindrical standoffs for a rectangular PCB
module pcb_standoffs(x_pitch, y_pitch, height, hole_dia, base_dia=6) {
    for (dx=[-1,1], dy=[-1,1])
        translate([dx*x_pitch/2, dy*y_pitch/2, 0])
            difference() {
                cylinder(h=height, d=base_dia);
                translate([0,0,-0.1]) cylinder(h=height+0.2, d=hole_dia);
            }
}

// Cut-out for a rectangular port on a wall
module port_cutout(w, h, depth=20) {
    translate([-w/2, -depth/2, -h/2])
        cube([w, depth, h]);
}

// Top removable grill — covers the WHOLE top face so the RPi fan AND the
// radar heat sink can breathe through a single removable piece.
module top_grill() {
    grill_w = case_w - 2*wall - 2*tol;
    grill_d = case_d - 2*wall - 2*tol;
    n_bars  = floor(grill_d / (grill_bar_w + grill_slot_w));
    difference() {
        // Plate + perimeter flange
        union() {
            translate([-grill_w/2, -grill_d/2, 0])
                cube([grill_w, grill_d, grill_thickness]);
            // Flange to keep the grill flush, sits on the rim
            translate([-grill_w/2 - grill_overlap, -grill_d/2 - grill_overlap, -grill_thickness*0.5])
                cube([grill_w + 2*grill_overlap, grill_d + 2*grill_overlap, grill_thickness*0.5]);
        }
        // Slots
        for (i=[0:n_bars-1]) {
            y = -grill_d/2 + i*(grill_bar_w + grill_slot_w) + grill_bar_w;
            translate([-grill_w/2 + 4, y, -1])
                cube([grill_w - 8, grill_slot_w, grill_thickness + 2]);
        }
    }
}

// Solid wedge for the radar (tilted by radar_tilt_deg around X axis)
module radar_shelf_wedge() {
    // A right-angled prism that sets the radar PCB at radar_tilt_deg
    h_front = 0.5;
    h_back  = radar_pcb_d * sin(radar_tilt_deg) + h_front;
    translate([-radar_pcb_w/2, -radar_pcb_d/2, 0])
        polyhedron(
            points = [
                [0,            0, h_front],
                [radar_pcb_w,  0, h_front],
                [radar_pcb_w,  radar_pcb_d, h_back],
                [0,            radar_pcb_d, h_back],
                [0,            0, 0],
                [radar_pcb_w,  0, 0],
                [radar_pcb_w,  radar_pcb_d, 0],
                [0,            radar_pcb_d, 0],
            ],
            faces = [
                [0,1,2,3],  // top (tilted)
                [4,5,1,0],  // front
                [7,6,5,4],  // bottom
                [3,2,6,7],  // back
                [0,3,7,4],  // left
                [1,5,6,2],  // right
            ]
        );
}

// ============================================================
// MAIN ASSEMBLY
// ============================================================

module case_body() {
    difference() {
        // Outer shell
        translate([-case_w/2, -case_d/2, 0])
            cube([case_w, case_d, case_h]);

        // Hollow inside (single big cavity; shelves and standoffs are added back)
        translate([-(case_w - 2*wall)/2, -(case_d - 2*wall)/2, wall])
            cube([case_w - 2*wall, case_d - 2*wall, case_h - 2*wall]);

        // ---- Back-panel cutouts (rear wall = +Y face) ----
        // The radar USB exits here too.
        back_y = case_d/2 + 0.1;

        // 4× USB-A on RPi (right long edge → maps to +X wall, not back)
        // To keep all "user" ports on the same back face, we route the RPi
        // long-edge ports through the +X (right) wall instead.
        right_x = case_w/2 + 0.1;

        // 4 USB-A stacks (two double-stacks → 4 holes total)
        for (i=[0,1])
            translate([right_x, usb_a_y_start + i*usb_a_y_pitch - rpi_pcb_d/2,
                       z_rpi_floor + rpi_pcb_t + 9])
                rotate([0,90,0]) port_cutout(usb_a_h, usb_a_w);

        // Ethernet
        translate([right_x, eth_y - rpi_pcb_d/2, z_rpi_floor + rpi_pcb_t + 10])
            rotate([0,90,0]) port_cutout(eth_h, eth_w);

        // Short-edge ports on RPi (front-short edge of PCB → maps to -X wall)
        left_x = -case_w/2 - 0.1;
        translate([left_x, usbc_x - rpi_pcb_w/2, z_rpi_floor + rpi_pcb_t + 3])
            rotate([0,90,0]) port_cutout(usbc_h, usbc_w);
        translate([left_x, hdmi0_x - rpi_pcb_w/2, z_rpi_floor + rpi_pcb_t + 3])
            rotate([0,90,0]) port_cutout(hdmi_h, hdmi_w);
        translate([left_x, hdmi1_x - rpi_pcb_w/2, z_rpi_floor + rpi_pcb_t + 3])
            rotate([0,90,0]) port_cutout(hdmi_h, hdmi_w);

        // Mission micro-USB cable exit (LEFT short edge of the case).
        // The micro-USB-B connector is centred on the short edge of the
        // radar PCB, at the average Z of the tilted PCB.
        translate([left_x, 0, z_radar_floor + radar_pcb_t + radar_usb_h/2 + 1])
            rotate([0, 90, 0])
                port_cutout(radar_usb_h + 3, radar_usb_w + 4);

        // I2C internal cable slot (camera → GPIO header). Inside path, no
        // external cutout needed; but we leave a small notch in the partition
        // between RPi compartment and camera shelf:
        translate([0, 0, z_grill - 1])
            cube([8, 6, 4], center=true);

        // Top grill opening — single rectangle that covers the whole top
        // face of the case so the removable grill plate sits flush over it.
        translate([0, 0, case_h - wall + 0.1])
            cube([case_w - 2*wall - 2*tol, case_d - 2*wall - 2*tol, wall + 1],
                 center=true);

        // Camera optical aperture (front-facing = -Y wall)
        translate([cam_sensor_offset_x, -case_d/2 - 0.1,
                   z_cam_floor + cam_shelf_h/2])
            rotate([-90, 0, 0]) cylinder(h=wall+1, d=cam_sensor_dia + 2);

        // Side ventilation slots (RPi compartment)
        for (side=[-1,1])
            for (i=[0:3])
                translate([side*(case_w/2 - wall/2),
                          -case_d/2 + 10 + i*(rpi_inner_d/4),
                          z_rpi_floor + 4])
                    cube([wall*2 + 1, 4, 12], center=true);
    }

    // Internal floor partition between RPi compartment and camera shelf
    translate([0, 0, z_grill])
        difference() {
            translate([-(case_w-2*wall)/2, -(case_d-2*wall)/2, 0])
                cube([case_w - 2*wall, case_d - 2*wall, wall]);
            // central opening for I2C cable
            translate([0, 0, -1]) cube([10, 6, wall+2], center=true);
        }

    // Standoffs for RPi (M2.5, raised)
    translate([0, 0, wall])
        pcb_standoffs(rpi_hole_x_pitch, rpi_hole_y_pitch,
                      rpi_clear_below, rpi_hole_dia);

    // Camera mount: 2 standoffs at the real asymmetric hole positions.
    // The PCB is centred so the sensor lines up with the front aperture
    // (which is offset by cam_sensor_offset_x). The two mounting holes
    // sit on the right side of the PCB.
    translate([-cam_sensor_offset_x, -cam_sensor_offset_y, z_cam_floor])
        for (p = [[cam_hole1_x, cam_hole1_y], [cam_hole2_x, cam_hole2_y]])
            translate([p[0], p[1], 0])
                difference() {
                    cylinder(h=cam_clear, d=5);
                    translate([0,0,-0.1])
                        cylinder(h=cam_clear+0.2, d=cam_hole_dia);
                }

    // Radar tilted shelf (wedge with M3 standoffs on top)
    translate([0, 0, z_radar_floor]) {
        radar_shelf_wedge();
        // Mount holes on the tilted top surface — placed at hole pitch
        translate([0, 0, 0])
            rotate([radar_tilt_deg, 0, 0])
                translate([0, 0, radar_pcb_d * sin(radar_tilt_deg) / 2 + 0.5])
                    pcb_standoffs(radar_hole_x, radar_hole_y, 4,
                                  radar_hole_dia, base_dia=7);
    }
}

// ============================================================
// PREVIEW
// ============================================================
// Render the case body and the grill side-by-side so you can see both.
// Comment out the grill translate() to print just the body.

case_body();

translate([case_w + 20, 0, 0])
    top_grill();
