// OpenSCAD file for Comic Sans Letter A with hollow circles
// Requirements:
// - Comic Sans letter A
// - Embedded in 1cm diameter hollow circle
// - Small hollow circle attached to top with 5mm hole

// Parameters
main_circle_diameter = 10; // 1cm = 10mm
main_circle_thickness = 1.2; // thickness of the main circle wall
small_circle_diameter = 6; // diameter of small circle at top
small_circle_thickness = 1.2; // thickness of small circle wall
hole_diameter = 5; // 5mm hole in small circle
letter_height = 2; // height/thickness of the letter
letter_size = 6; // size of the letter
letter_to_draw = "A"; // letter to draw - can be changed to any letter

// Main module - can now draw any letter
module letter_pendant(letter = "A") {
    union() {
        // Main hollow circle
        difference() {
            cylinder(h = letter_height, d = main_circle_diameter, center = true);
            cylinder(h = letter_height + 0.1, d = main_circle_diameter - 2 * main_circle_thickness, center = true);
        }
        
        // Letter (parameterized - can be any letter)
        translate([0,- 0.6, 0]) {
            draw_letter(letter, letter_size, letter_height);
        }
        
        // Small hollow circle attached to top with rounded connection
        translate([0, main_circle_diameter/2 + small_circle_diameter/2 - 1, 0]) {
            difference() {
                union() {
                    // Main small circle
                    cylinder(h = letter_height, d = small_circle_diameter, center = true);
                    
                    // Rounded connection/fillet between small and main circles
                    hull() {
                        // Small cylinder at edge of main circle
                        translate([0, -small_circle_diameter/2 + 1, 0])
                            cylinder(h = letter_height, d = 2, center = true);
                        // Small cylinder at edge of small circle
                        translate([0, -small_circle_diameter/4, 0])
                            cylinder(h = letter_height, d = 1.5, center = true);
                    }
                }
                // Hole through both the circle and the fillet
                cylinder(h = letter_height + 0.1, d = hole_diameter, center = true);
            }
        }
    }
}

// Parameterized letter function
module draw_letter(letter = "A", size = 6, height = 2) {
    // Use text function for any letter with Comic Sans styling
    linear_extrude(height = height, center = true) {
        text(letter, size = size, font = "Comic Sans MS:style=Regular", 
             halign = "center", valign = "center");
    }
}

// Fallback module for letter A (if Comic Sans font is not available)
module letter_a_fallback() {
    difference() {
        union() {
            // Main body of letter A using simplified geometry
            hull() {
                // Left leg
                translate([-letter_size/3, -letter_size/3, 0])
                    cylinder(h = letter_height, d = 1.2, center = true);
                translate([-letter_size/6, letter_size/3, 0])
                    cylinder(h = letter_height, d = 1.2, center = true);
            }
            
            hull() {
                // Right leg
                translate([letter_size/3, -letter_size/3, 0])
                    cylinder(h = letter_height, d = 1.2, center = true);
                translate([letter_size/6, letter_size/3, 0])
                    cylinder(h = letter_height, d = 1.2, center = true);
            }
            
            // Crossbar
            hull() {
                translate([-letter_size/6, -letter_size/8, 0])
                    cylinder(h = letter_height, d = 1.0, center = true);
                translate([letter_size/6, -letter_size/8, 0])
                    cylinder(h = letter_height, d = 1.0, center = true);
            }
            
            // Top peak with rounded Comic Sans style
            translate([0, letter_size/3, 0])
                cylinder(h = letter_height, d = 2, center = true);
        }
        
        // Remove center area to make it more letter-like
        translate([0, 0, 0])
            cylinder(h = letter_height + 0.1, d = letter_size/4, center = true);
    }
}

// Alternative: More detailed Comic Sans Letter A using text()
// Uncomment the module below if you prefer using OpenSCAD's text function
// (requires a Comic Sans font to be installed on your system)

/*
module comic_sans_letter_a() {
    linear_extrude(height = letter_height, center = true) {
        text("A", size = letter_size, font = "Comic Sans MS:style=Regular", 
             halign = "center", valign = "center");
    }
}
*/

// Parameters for layout
spacing = 20; // spacing between letter pendants (increased to prevent touching)
letters_to_generate = ["A", "B", "C", "D", "E", "F", "G"]; // letters A through G

// Module to create a square layout of letters A through G
module letters_square_layout() {
    // Calculate positions for a square-ish layout
    // For 7 letters, we'll use a 3x3 grid with empty spots
    grid_size = 3;
    
    for (i = [0 : len(letters_to_generate) - 1]) {
        // Calculate row and column for current letter
        row = floor(i / grid_size);
        col = i % grid_size;
        
        // Calculate position
        x_pos = (col - 1) * spacing; // center the grid
        y_pos = (1 - row) * spacing; // center the grid (inverted Y)
        
        translate([x_pos, y_pos, 0]) {
            letter_pendant(letters_to_generate[i]);
        }
    }
}

// Render the letters A through G in a square layout
letters_square_layout();

// Alternative: Render a single letter (uncomment and change letter as needed)
// letter_pendant(letter_to_draw);

// Examples: Uncomment one of these lines to try different letters
// letter_pendant("B");
// letter_pendant("C");
// letter_pendant("1");
// letter_pendant("♥");  // Works with special characters too (if font supports them)

// Preview settings for better visualization
$fn = 50; // smooth circles