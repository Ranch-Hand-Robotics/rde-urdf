// Solar System Model in OpenSCAD

// Parameters
planet_scale = 1.0; // Scale for planet sizes (in mm)
orbit_scale = 1.0;  // Scale for orbit distances (in mm)

// Colors for planets
colors = [
    [1, 1, 0],       // Sun (Yellow)
    [0.5, 0.5, 0.5], // Mercury (Gray)
    [1, 0.5, 0],     // Venus (Orange)
    [0, 0, 1],       // Earth (Blue)
    [1, 0, 0],       // Mars (Red)
    [1, 1, 1],       // Jupiter (White)
    [1, 0.8, 0.5],   // Saturn (Light Orange)
    [0, 1, 1],       // Uranus (Cyan)
    [0, 0, 0.5]      // Neptune (Dark Blue)
];

// Radii of planets (in mm)
radii = [50, 10, 20, 21, 15, 40, 35, 30, 28];

// Orbital distances (in mm)
orbits = [0, 100, 150, 200, 250, 350, 450, 550, 650];

// Function to create a planet
module planet(radius, color) {
    color(color)
        sphere(r=radius * planet_scale);
}

// Function to create an orbit ring
module orbit(distance) {
    if (distance > 0) {
        color([0.5, 0.5, 0.5, 0.3]) // Semi-transparent gray orbit
            difference() {
                cylinder(h=1, r=distance * orbit_scale + 2, center=true);
                cylinder(h=2, r=distance * orbit_scale - 2, center=true);
            }
    }
}

// Solar System
module solar_system() {
    for (i = [0 : len(radii) - 1]) {
        orbit(orbits[i]);
        translate([orbits[i] * orbit_scale, 0, 0])
            planet(radii[i], colors[i]);
    }
}

// Render the Solar System
solar_system();