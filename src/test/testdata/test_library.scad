// Simple OpenSCAD test file that uses a library function
// This tests whether library support is working

include <MCAD/boxes.scad>

// Create a rounded box using MCAD library
roundedBox([10, 20, 5], 1, true);

// If MCAD library is not available, fall back to simple cube
// cube([10, 20, 5]);