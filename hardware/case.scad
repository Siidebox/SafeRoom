// SafeRoom case v2 — 2026-05-14
// IWR6843AOPEVM (on TI/Mistral L-bracket) + MLX90640 IR camera + Raspberry Pi 5
//
// Three stacked compartments (bottom -> top):
//   1. RPi 5 + Armor Lite, all ports through side walls
//   2. MLX90640 camera on a flat shelf (tilt parameterised, default 0°)
//   3. Radar bracket+EVM glued onto a 10° wedge, USB cable exits side wall
//
// Top opens via a removable slotted grill.
//
// Print orientation: case upright as drawn.

// ============================================================
// CONFIG
// ============================================================

// --- Wall / tolerance ----------------------------------
wall            = 2.4;
tol             = 0.4;
$fn             = 48;

// --- Radar bracket envelope (user-provided, includes margin) -------
// The radar sits on its TI/Mistral L-bracket. The whole assembly fits
// inside this parallelepiped, with 7 mm of extra length on the +X side
// (unused USB / debug pins). Glued onto the wedge with double-sided
// industrial tape.
bracket_w       = 122;    // X (length, includes 7 mm margin on +X side)
bracket_d       = 50;     // Y (depth)
bracket_h       = 50;     // Z (height of bracket+radar stack)
radar_tilt_deg  = 10;     // wedge tilt (front-down)

// AOP chip position inside the bracket envelope. The radar PCB sits at
// the -X end of the envelope (7 mm gap left on +X). Chip is offset
// −20.12 mm from the PCB X centre (STEP-measured).
//   PCB X centre in bracket: −bracket_w/2 + 7 + radar_pcb_w/2
//                          = −61 + 7 + 57.635 = −3.365 mm
//   chip X in bracket: −3.365 + (−20.12) = −23.48 mm
aperture_offset_x = -23.5;   // X of aperture in case coords
aperture_w        = 20.0;    // antenna window width
aperture_h        = 20.0;    // antenna window height

// USB cable exit slot for the radar (on +X wall of radar compartment).
// Cable comes out and plugs into one of the RPi USB-A ports externally.
usb_slot_w        = 12;   // along Y
usb_slot_h        = 8;    // along Z

// --- MLX90640 camera ----------------------------------
cam_pcb_w           = 28.0;
cam_pcb_d           = 16.0;
cam_pcb_t           = 1.6;
cam_hole_dia        = 2.4;
cam_hole1_x         = 12.0;
cam_hole1_y         = -6.0;
cam_hole2_x         = 12.0;
cam_hole2_y         =  6.0;
cam_sensor_offset_x = 5.5;
cam_sensor_dia      = 10.0;     // optical aperture
cam_tilt_deg        = 0;        // open question — change after FOV test
cam_clear           = 8;        // generous margin around the camera PCB

// --- Raspberry Pi 5 + Armor Lite V5 ------------------
rpi_pcb_w           = 85.0;
rpi_pcb_d           = 56.0;
rpi_pcb_t           = 1.6;
rpi_hole_dia        = 2.8;
rpi_hole_x_pitch    = 58.0;
rpi_hole_y_pitch    = 49.0;
armor_height        = 16.0;

// Port positions on RPi 5
usb_a_y_start       = 9.0;
usb_a_y_pitch       = 18.0;
usb_a_h             = 16.0;
usb_a_w             = 13.5;
eth_y               = 45.0;
eth_w               = 16.0;
eth_h               = 14.0;
usbc_x              = 11.0;
usbc_w              = 9.0;
usbc_h              = 4.0;
hdmi0_x             = 26.0;
hdmi1_x             = 39.5;
hdmi_w              = 7.5;
hdmi_h              = 4.0;

// --- Clearances --------------------------------------
rpi_clear_top       = 6.0;
rpi_clear_below     = 6.0;
rpi_clear_sides     = 6.0;
radar_clear_around  = 5.0;
shelf_gap           = 4.0;

// --- Top grill ----------------------------------------
grill_thickness     = 2.4;
grill_bar_w         = 2.0;
grill_slot_w        = 4.0;
grill_overlap       = 4.0;

// ============================================================
// DERIVED
// ============================================================

rpi_inner_w = rpi_pcb_w + 2 * rpi_clear_sides;
rpi_inner_d = rpi_pcb_d + 2 * rpi_clear_sides;
rpi_inner_h = rpi_clear_below + rpi_pcb_t + armor_height + rpi_clear_top;

cam_inner_w = max(cam_pcb_w + 2 * cam_clear, 60);
cam_inner_d = max(cam_pcb_d + 2 * cam_clear, rpi_inner_d);
cam_compartment_h = 25;

radar_inner_w = bracket_w + 2 * radar_clear_around;
radar_inner_d = bracket_d + 2 * radar_clear_around;
wedge_max_h   = bracket_d * sin(radar_tilt_deg);     // ~8.7 mm
radar_inner_h = wedge_max_h + bracket_h + 4;         // 4 mm headroom

case_w = max(radar_inner_w, cam_inner_w, rpi_inner_w) + 2 * wall;
case_d = max(radar_inner_d, cam_inner_d, rpi_inner_d) + 2 * wall;
case_h = wall + rpi_inner_h + shelf_gap + cam_compartment_h
              + shelf_gap + radar_inner_h + wall;

z_rpi_floor   = wall + rpi_clear_below;
z_rpi_top     = z_rpi_floor + rpi_pcb_t + armor_height;
z_partition   = z_rpi_top + rpi_clear_top;
z_cam_floor   = z_partition + shelf_gap;
z_radar_floor = z_cam_floor + cam_compartment_h + shelf_gap;

echo("Case outer:", case_w, "x", case_d, "x", case_h, "mm");

// ============================================================
// MODULES
// ============================================================

module pcb_standoffs(x_pitch, y_pitch, height, hole_dia, base_dia=6) {
    for (dx = [-1, 1], dy = [-1, 1])
        translate([dx * x_pitch / 2, dy * y_pitch / 2, 0])
            difference() {
                cylinder(h = height, d = base_dia);
                translate([0, 0, -0.1])
                    cylinder(h = height + 0.2, d = hole_dia);
            }
}

module port_cutout(w, h, depth = 20) {
    translate([-w / 2, -depth / 2, -h / 2])
        cube([w, depth, h]);
}

module top_grill() {
    gw = case_w - 2 * wall - 2 * tol;
    gd = case_d - 2 * wall - 2 * tol;
    n  = floor(gd / (grill_bar_w + grill_slot_w));
    difference() {
        union() {
            translate([-gw / 2, -gd / 2, 0])
                cube([gw, gd, grill_thickness]);
            translate([-gw / 2 - grill_overlap,
                       -gd / 2 - grill_overlap,
                       -grill_thickness * 0.5])
                cube([gw + 2 * grill_overlap,
                      gd + 2 * grill_overlap,
                      grill_thickness * 0.5]);
        }
        for (i = [0 : n - 1]) {
            y = -gd / 2 + i * (grill_bar_w + grill_slot_w) + grill_bar_w;
            translate([-gw / 2 + 4, y, -1])
                cube([gw - 8, grill_slot_w, grill_thickness + 2]);
        }
    }
}

// Flat-top 10° wedge for the radar bracket. Bracket glues on top with
// double-sided industrial tape. Low edge at -Y (front), high edge at +Y
// (back) so the bracket leans forward by radar_tilt_deg.
module radar_wedge() {
    h_lo = 1;
    h_hi = bracket_d * sin(radar_tilt_deg) + h_lo;
    translate([-bracket_w / 2, -bracket_d / 2, 0])
        polyhedron(
            points = [
                [0,         0,          0   ],
                [bracket_w, 0,          0   ],
                [bracket_w, bracket_d,  0   ],
                [0,         bracket_d,  0   ],
                [0,         0,          h_lo],
                [bracket_w, 0,          h_lo],
                [bracket_w, bracket_d,  h_hi],
                [0,         bracket_d,  h_hi]
            ],
            faces = [
                [3, 2, 1, 0],
                [4, 5, 6, 7],
                [0, 1, 5, 4],
                [2, 3, 7, 6],
                [1, 2, 6, 5],
                [3, 0, 4, 7]
            ]
        );
}

// Camera shelf — flat plate or tilted wedge depending on cam_tilt_deg.
module cam_shelf() {
    bw = cam_pcb_w + 2 * 5;
    bd = cam_pcb_d + 2 * 5;
    if (cam_tilt_deg == 0) {
        translate([-bw / 2, -bd / 2, 0])
            cube([bw, bd, 2]);
    } else {
        h_lo = 1;
        h_hi = bd * sin(cam_tilt_deg) + h_lo;
        translate([-bw / 2, -bd / 2, 0])
            polyhedron(
                points = [
                    [0,  0,  0   ], [bw, 0,  0   ],
                    [bw, bd, 0   ], [0,  bd, 0   ],
                    [0,  0,  h_lo], [bw, 0,  h_lo],
                    [bw, bd, h_hi], [0,  bd, h_hi]
                ],
                faces = [
                    [3, 2, 1, 0], [4, 5, 6, 7],
                    [0, 1, 5, 4], [2, 3, 7, 6],
                    [1, 2, 6, 5], [3, 0, 4, 7]
                ]
            );
    }
}

// ============================================================
// MAIN ASSEMBLY
// ============================================================

module case_body() {
    difference() {
        // Outer shell
        translate([-case_w / 2, -case_d / 2, 0])
            cube([case_w, case_d, case_h]);

        // Inner cavity
        translate([-(case_w - 2 * wall) / 2,
                   -(case_d - 2 * wall) / 2,
                    wall])
            cube([case_w - 2 * wall,
                  case_d - 2 * wall,
                  case_h - 2 * wall]);

        right_x = case_w / 2 + 0.1;
        left_x  = -case_w / 2 - 0.1;
        front_y = -case_d / 2 - 0.1;

        // --- RPi short edge (-X wall): USB-C + 2x micro-HDMI ---
        translate([left_x, usbc_x - rpi_pcb_w / 2,
                   z_rpi_floor + rpi_pcb_t + 3])
            rotate([0, 90, 0]) port_cutout(usbc_h, usbc_w);
        translate([left_x, hdmi0_x - rpi_pcb_w / 2,
                   z_rpi_floor + rpi_pcb_t + 3])
            rotate([0, 90, 0]) port_cutout(hdmi_h, hdmi_w);
        translate([left_x, hdmi1_x - rpi_pcb_w / 2,
                   z_rpi_floor + rpi_pcb_t + 3])
            rotate([0, 90, 0]) port_cutout(hdmi_h, hdmi_w);

        // --- RPi long edge (+X wall): 4x USB-A + Ethernet ---
        for (i = [0, 1])
            translate([right_x,
                       usb_a_y_start + i * usb_a_y_pitch - rpi_pcb_d / 2,
                       z_rpi_floor + rpi_pcb_t + 9])
                rotate([0, 90, 0]) port_cutout(usb_a_h, usb_a_w);
        translate([right_x, eth_y - rpi_pcb_d / 2,
                   z_rpi_floor + rpi_pcb_t + 10])
            rotate([0, 90, 0]) port_cutout(eth_h, eth_w);

        // --- Camera optical aperture (front -Y wall) ---
        translate([cam_sensor_offset_x, front_y,
                   z_cam_floor + cam_compartment_h / 2])
            rotate([-90, 0, 0])
                cylinder(h = wall + 1, d = cam_sensor_dia);

        // --- Radar antenna aperture (front -Y wall) ---
        // X centred over the AOP chip; Z roughly at mid-bracket height
        // (bracket holds the radar vertically; chip sits in the upper half).
        translate([aperture_offset_x, front_y,
                   z_radar_floor + bracket_h * 0.6])
            rotate([-90, 0, 0])
                translate([-aperture_w / 2, -aperture_h / 2, 0])
                    cube([aperture_w, aperture_h, wall + 1]);

        // --- Radar USB cable exit slot (+X wall, radar compartment) ---
        translate([right_x, 0,
                   z_radar_floor + bracket_h * 0.45])
            rotate([0, 90, 0])
                translate([-usb_slot_h / 2, -usb_slot_w / 2, 0])
                    cube([usb_slot_h, usb_slot_w, wall + 1]);

        // --- Top opening for the grill ---
        translate([0, 0, case_h - wall + 0.1])
            cube([case_w - 2 * wall - 2 * tol,
                  case_d - 2 * wall - 2 * tol,
                  wall + 1],
                 center = true);

        // --- RPi compartment side ventilation slots ---
        for (side = [-1, 1])
            for (i = [0 : 3])
                translate([side * (case_w / 2 - wall / 2),
                           -case_d / 2 + 12 + i * (rpi_inner_d / 4),
                           z_rpi_floor + 4])
                    cube([wall * 2 + 1, 4, 12], center = true);
    }

    // --- Internal floor between RPi and sensor stack ---
    translate([-(case_w - 2 * wall) / 2,
               -(case_d - 2 * wall) / 2,
                z_partition])
        difference() {
            cube([case_w - 2 * wall, case_d - 2 * wall, wall]);
            // Dupont cable notch (camera I2C/UART to RPi GPIO)
            translate([(case_w - 2 * wall) / 2,
                       (case_d - 2 * wall) / 2,
                       -1])
                cube([14, 10, wall + 2], center = true);
        }

    // --- RPi standoffs ---
    translate([0, 0, wall])
        pcb_standoffs(rpi_hole_x_pitch, rpi_hole_y_pitch,
                      rpi_clear_below, rpi_hole_dia);

    // --- Camera shelf + M2 standoffs ---
    translate([-cam_sensor_offset_x, 0, z_cam_floor]) {
        cam_shelf();
        z_so = (cam_tilt_deg == 0) ? 2
             : (cam_pcb_d * sin(cam_tilt_deg) / 2 + 1);
        translate([0, 0, z_so])
            for (p = [[cam_hole1_x, cam_hole1_y],
                      [cam_hole2_x, cam_hole2_y]])
                translate([p[0], p[1], 0])
                    difference() {
                        cylinder(h = 4, d = 5);
                        translate([0, 0, -0.1])
                            cylinder(h = 4.2, d = cam_hole_dia);
                    }
    }

    // --- Radar wedge (bracket glues on top) ---
    translate([0, 0, z_radar_floor])
        radar_wedge();
}

// ============================================================
// PREVIEW
// ============================================================

case_body();

translate([case_w + 20, 0, 0])
    top_grill();
