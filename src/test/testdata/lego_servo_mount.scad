// LEGO Compatible Servo Mount
// This file creates a LEGO-compatible mount for standard servo motors
// Uses MCAD library for LEGO compatibility and servo dimensions

include <MCAD/lego_compatibility.scad>

// Parameters for TowerPro SG90 micro servo (in mm)
servo_width = 22.5;
servo_length = 12;
servo_height = 22.8;
servo_tab_width = 32;
servo_tab_thickness = 2.5;
servo_tab_hole_spacing = 28;
servo_tab_hole_diameter = 2;

// LEGO base parameters
lego_base_width = 4;  // LEGO studs wide
lego_base_length = 6; // LEGO studs long
lego_base_height = 1; // LEGO brick height units (3.2mm each)

// Mount parameters (in mm)
mount_height = 10;    // Height for servo clearance in mm
wall_thickness = 2;   // Wall thickness around servo
tolerance = 0.5;      // Clearance for servo fit

// Main LEGO servo mount module
module lego_servo_mount() {
    difference() {
        union() {
            // LEGO base plate with reinforcement
            translate([0, 0, lego_base_height * 3.2 / 2])
                block(lego_base_width, lego_base_length, lego_base_height, 
                      reinforcement=true, center=true);
            
            // Top mounting plate for servo tabs
            translate([0, 0, lego_base_height * 3.2 + wall_thickness/2])
                cube([servo_tab_width, 
                      servo_length + 2 * wall_thickness,
                      wall_thickness], center=true);
        }
        
        // Servo body cavity - cut completely through the LEGO base
        translate([0, 0, lego_base_height * 3.2 / 2])
            cube([servo_width + tolerance, 
                  servo_length + tolerance,
                  lego_base_height * 3.2 + 2], center=true);
        
        // Servo tab mounting holes
        translate([0, 0, lego_base_height * 3.2 + wall_thickness/2]) {
            for (x = [-servo_tab_hole_spacing/2, servo_tab_hole_spacing/2]) {
                translate([x, 0, 0])
                    cylinder(h = wall_thickness + 1, 
                            d = servo_tab_hole_diameter, 
                            center = true, $fn = 20);
            }
        }
        
        // Wire access slot - cut through the side
        translate([0, -(servo_length + tolerance)/2 - 1, 
                   lego_base_height * 3.2 - servo_height/2 + 2])
            cube([8, 4, servo_height - 4], center=true);
    }
}

// Alternative version with adjustable parameters
module adjustable_lego_servo_mount(base_w = 4, base_l = 6, mount_h = 10) {
    difference() {
        union() {
            // LEGO base plate
            translate([0, 0, lego_base_height * 3.2 / 2])
                block(base_w, base_l, lego_base_height, 
                      reinforcement=true, center=true);
            
            // Servo mounting structure with rounded corners
            translate([0, 0, lego_base_height * 3.2 + mount_h / 2])
                hull() {
                    for (x = [-(servo_width + 2 * wall_thickness)/2 + 2, 
                              (servo_width + 2 * wall_thickness)/2 - 2]) {
                        for (y = [-(servo_length + 2 * wall_thickness)/2 + 2, 
                                  (servo_length + 2 * wall_thickness)/2 - 2]) {
                            translate([x, y, 0])
                                cylinder(h = mount_h, r = 2, center = true, $fn = 20);
                        }
                    }
                }
            
            // Top mounting plate
            translate([0, 0, lego_base_height * 3.2 + mount_h + wall_thickness/2])
                hull() {
                    for (x = [-servo_tab_width/2 + 2, servo_tab_width/2 - 2]) {
                        for (y = [-(servo_length + 2 * wall_thickness)/2 + 2, 
                                  (servo_length + 2 * wall_thickness)/2 - 2]) {
                            translate([x, y, 0])
                                cylinder(h = wall_thickness, r = 2, center = true, $fn = 20);
                        }
                    }
                }
        }
        
        // Servo cavity with rounded corners for easier insertion
        translate([0, 0, lego_base_height * 3.2 + mount_h / 2])
            hull() {
                for (x = [-(servo_width + tolerance)/2 + 1, 
                          (servo_width + tolerance)/2 - 1]) {
                    for (y = [-(servo_length + tolerance)/2 + 1, 
                              (servo_length + tolerance)/2 - 1]) {
                        translate([x, y, 0])
                            cylinder(h = mount_h + 1, 
                                   r = 1, center = true, $fn = 20);
                    }
                }
            }
        
        // Servo tab mounting holes with countersink
        translate([0, 0, lego_base_height * 3.2 + mount_h + wall_thickness/2]) {
            for (x = [-servo_tab_hole_spacing/2, servo_tab_hole_spacing/2]) {
                translate([x, 0, 0]) {
                    // Main hole
                    cylinder(h = wall_thickness + 1, 
                            d = servo_tab_hole_diameter, 
                            center = true, $fn = 20);
                    // Countersink for screw heads
                    translate([0, 0, wall_thickness/2])
                        cylinder(h = wall_thickness/2 + 0.1, 
                                d1 = servo_tab_hole_diameter, 
                                d2 = servo_tab_hole_diameter + 2, 
                                center = true, $fn = 20);
                }
            }
        }
        
        // Improved wire access channel
        translate([0, -(servo_length + 2 * wall_thickness)/2, 
                   lego_base_height * 3.2 + mount_h / 3])
            hull() {
                cube([8, wall_thickness + 1, mount_h * 2/3], center=true);
                translate([0, -2, 0])
                    cube([6, 1, mount_h * 2/3], center=true);
            }
    }
}

// Side-mount version for horizontal servo orientation
module lego_servo_mount_horizontal() {
    difference() {
        union() {
            // LEGO base plate
            translate([0, 0, lego_base_height * 3.2 / 2])
                block(lego_base_width, lego_base_length + 2, lego_base_height, 
                      reinforcement=true, center=true);
            
            // Vertical mounting wall
            translate([0, 0, lego_base_height * 3.2 + servo_height / 2])
                cube([servo_length + 2 * wall_thickness, 
                      servo_width + 2 * wall_thickness,
                      servo_height], center=true);
            
            // Side mounting tabs
            for (y = [-(servo_width + 2 * wall_thickness)/2 - servo_tab_thickness/2,
                      (servo_width + 2 * wall_thickness)/2 + servo_tab_thickness/2]) {
                translate([0, y, lego_base_height * 3.2 + servo_height - wall_thickness])
                    cube([servo_tab_width, servo_tab_thickness, wall_thickness * 2], 
                         center=true);
            }
        }
        
        // Servo cavity
        translate([0, 0, lego_base_height * 3.2 + servo_height / 2])
            cube([servo_length + tolerance, 
                  servo_width + tolerance,
                  servo_height + 1], center=true);
        
        // Side tab mounting holes
        for (y = [-(servo_width + 2 * wall_thickness)/2 - servo_tab_thickness/2,
                  (servo_width + 2 * wall_thickness)/2 + servo_tab_thickness/2]) {
            for (x = [-servo_tab_hole_spacing/2, servo_tab_hole_spacing/2]) {
                translate([x, y, lego_base_height * 3.2 + servo_height - wall_thickness])
                    cylinder(h = wall_thickness * 2 + 1, 
                            d = servo_tab_hole_diameter, 
                            center = true, $fn = 20);
            }
        }
        
        // Wire access
        translate([-(servo_length + 2 * wall_thickness)/2, 0, 
                   lego_base_height * 3.2 + servo_height/4])
            cube([wall_thickness + 1, 6, servo_height/2], center=true);
    }
}

// Demo showing different mount types
module demo_mounts() {
    // Standard vertical mount
    translate([-40, 0, 0])
        lego_servo_mount();
    
    // Adjustable mount with larger base
    translate([0, 0, 0])
        adjustable_lego_servo_mount(4, 6, 12);
    
    // Horizontal mount
    translate([40, 0, 0])
        lego_servo_mount_horizontal();
}

// Render a single servo mount
lego_servo_mount();

// Alternative mounts (uncomment as needed)
// adjustable_lego_servo_mount(4, 6, 10);
// lego_servo_mount_horizontal();
// demo_mounts(); // Show all three designs

// Settings for smooth rendering
$fn = 50;