/*
 * Test Robot Components Library
 * 
 * This library provides basic components for building robots
 * including chassis, wheels, and sensors.
 * 
 * Author: Test User
 * Version: 1.0
 */

// Basic chassis module for robots
// Creates a rectangular chassis with mounting holes
module robot_chassis(width=100, length=150, height=20, hole_diameter=3) {
    difference() {
        // Main chassis body
        cube([length, width, height], center=true);
        
        // Corner mounting holes
        for (x = [-length/2 + 10, length/2 - 10]) {
            for (y = [-width/2 + 10, width/2 - 10]) {
                translate([x, y, 0])
                    cylinder(h=height+1, d=hole_diameter, center=true);
            }
        }
    }
}

// Simple wheel module
// Creates a basic wheel with optional tire tread
module robot_wheel(diameter=60, width=20, hub_diameter=6, with_tread=true) {
    difference() {
        union() {
            // Main wheel
            cylinder(h=width, d=diameter, center=true);
            
            // Add tread pattern if requested
            if (with_tread) {
                for (angle = [0:30:330]) {
                    rotate([0, 0, angle])
                        translate([diameter/2 - 2, 0, 0])
                            cube([4, 2, width], center=true);
                }
            }
        }
        
        // Hub hole
        cylinder(h=width+1, d=hub_diameter, center=true);
    }
}

// Ultrasonic sensor mount
// Creates a mount for HC-SR04 ultrasonic sensor
function sensor_mount_holes() = [[21, 0], [-21, 0]];

module ultrasonic_sensor_mount(thickness=3, mount_height=15) {
    difference() {
        // Main mount body
        cube([50, 20, thickness], center=true);
        
        // Sensor mounting holes
        for (pos = sensor_mount_holes()) {
            translate([pos[0], pos[1], 0])
                cylinder(h=thickness+1, d=2, center=true);
        }
    }
    
    // Vertical support
    translate([0, -10, mount_height/2])
        cube([50, thickness, mount_height], center=true);
}