// Complex OpenSCAD test file
// Creates a robot-like structure

module wheel(radius=.10, width=.05) {
    cylinder(h=width, r=radius, center=true, $fn=100);
}

module robot_base() {
    // Main body
    cube([.40, .30, .20], center=true);
    
    // Wheels - rotated 90 degrees around X-axis to orient them properly for rolling
    translate([-.15, -.20, -.15]) rotate([90, 0, 0]) wheel();
    translate([.15, -.20, -.15]) rotate([90, 0, 0]) wheel();
    translate([-.15, .20, -.15]) rotate([90, 0, 0]) wheel();
    translate([.15, .20, -.15]) rotate([90, 0, 0]) wheel();
}

// Create the robot
robot_base();