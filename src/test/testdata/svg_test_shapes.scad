// Test file for SVG export functionality
// This file contains 2D shapes that can be exported to SVG

// Create a circle
circle(r=15);

// Create a square
translate([40, 0, 0]) 
  square([25, 25]);

// Create a polygon (triangle)
translate([80, 0, 0]) 
  polygon(points=[[0,0],[20,0],[10,20]]);

// Create text (if supported)
translate([0, -40, 0])
  text("SVG Test", size=8);
