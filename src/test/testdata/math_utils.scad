// Math utilities for OpenSCAD
// Provides useful mathematical functions and constants

// Calculate distance between two 2D points
function distance_2d(p1, p2) = sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));

// Calculate distance between two 3D points
function distance_3d(p1, p2) = sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) + pow(p2[2] - p1[2], 2));

/* 
   Linear interpolation between two values
   t should be between 0 and 1
   Returns a + t * (b - a)
*/
function lerp(a, b, t) = a + t * (b - a);

// Clamp value between min and max
function clamp(value, min_val, max_val) = max(min_val, min(max_val, value));

// Convert degrees to radians
function deg_to_rad(degrees) = degrees * PI / 180;

// Convert radians to degrees  
function rad_to_deg(radians) = radians * 180 / PI;