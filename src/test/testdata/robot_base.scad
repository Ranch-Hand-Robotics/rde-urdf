// Complex OpenSCAD test file
// Creates a robot-like structure

module wheel(radius=100, width=50) {
    cylinder(h=width, r=radius, center=true, $fn=100);
}

module robot_base() {
    // Main body
    cube([400, 300, 200], center=true);
    
    // Wheels - rotated 90 degrees around X-axis to orient them properly for rolling
    translate([-150, -200, -150]) rotate([90, 0, 0]) wheel();
    translate([150, -200, -150]) rotate([90, 0, 0]) wheel();
    translate([-150, 200, -150]) rotate([90, 0, 0]) wheel();
    translate([150, 200, -150]) rotate([90, 0, 0]) wheel();
}

// Create the robot
robot_base();