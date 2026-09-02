/*
 * Rubber Duck Debugger
 *
 * A joyful OpenSCAD stress test for colors, debug modifiers, and the Customizer.
 * The duck catches literal bugs before they reach the disabled "production" node.
 * Units: millimeters
 */

/* [Duck DNA] */
// Overall mascot size
Duck_scale = 1.0; // [0.75:0.05:1.50]

// Choose the duck's debugging personality
Mood = "Found a bug"; // [Found a bug,Thinking,Ship it]

// Pick a built-in color story
Palette = "Cyber"; // [Cyber,Classic,Sunset,Monochrome]

// Number of bugs orbiting the duck
Bug_count = 3; // [1:1:8]

/* [Party Mode] */
// Add rainbow tail feathers
Rainbow_tail = true;

// Add a tiny crown for exceptionally clean builds
Ship_it_crown = true;

// User-editable RGB accent color
Accent_color = [0.10, 0.95, 1.00]; // [0:0.05:1]

/* [Debug Operators] */
// Change this to inspect the model's debugging layers
Debug_view = "Highlights"; // [Off,Highlights,Ghost shell,X-ray]

// Show the translucent build-volume guide
Show_build_volume = false;

/* [Hidden] */
$fn = 48;
EPS = 0.01;
rainbow = [
    [1.00, 0.15, 0.25],
    [1.00, 0.50, 0.05],
    [1.00, 0.90, 0.10],
    [0.10, 0.85, 0.35],
    [0.10, 0.55, 1.00],
    [0.65, 0.20, 1.00]
];

function duck_color() =
    Palette == "Classic" ? [1.00, 0.78, 0.05] :
    Palette == "Sunset" ? [1.00, 0.28, 0.18] :
    Palette == "Monochrome" ? [0.82, 0.84, 0.88] :
    [0.17, 0.95, 0.82];

function wing_color() =
    Palette == "Classic" ? [1.00, 0.63, 0.02] :
    Palette == "Sunset" ? [0.72, 0.08, 0.42] :
    Palette == "Monochrome" ? [0.35, 0.38, 0.45] :
    [0.45, 0.12, 0.95];

function beak_color() = Palette == "Monochrome" ? [0.25, 0.27, 0.30] : [1.00, 0.32, 0.04];
function base_color() = Palette == "Monochrome" ? [0.12, 0.13, 0.15] : [0.025, 0.035, 0.09];

module ellipsoid(size = [10, 10, 10]) {
    scale(size / 2) sphere(r = 1);
}

module rounded_bar(length, width, height) {
    hull() {
        translate([-(length - width) / 2, 0, 0]) cylinder(h = height, d = width, center = true);
        translate([ (length - width) / 2, 0, 0]) cylinder(h = height, d = width, center = true);
    }
}

module code_pedestal() {
    color(base_color())
        union() {
            cylinder(h = 7, r = 34, $fn = 12);
            translate([0, 0, 6]) cylinder(h = 3, r1 = 30, r2 = 27, $fn = 12);
        }

    // Glowing angle brackets turn the base into a tiny code terminal.
    color(Accent_color)
        for (side = [-1, 1])
            translate([side * 16, -29.5, 4.5])
                rotate([90, 0, 0])
                    linear_extrude(height = 1.2, center = true)
                        polygon(points = side < 0
                            ? [[4, 7], [-4, 0], [4, -7], [7, -7], [-1, 0], [7, 7]]
                            : [[-4, 7], [4, 0], [-4, -7], [-7, -7], [1, 0], [-7, 7]]);

    color([0.35, 0.38, 0.46])
        for (x = [-20, 0, 20])
            translate([x, 0, 8.5]) cylinder(h = 1.5, d = 3.5, center = true);
}

module duck_body_shell(body_color = duck_color()) {
    color(body_color) {
        translate([0, 1, 34]) ellipsoid([44, 36, 48]);
        translate([0, -2, 67]) ellipsoid([35, 32, 34]);
    }
}

module duck_wing(side = 1) {
    color(wing_color())
        translate([side * 21, 0, 38])
            rotate([0, side * 24, side * 10])
                ellipsoid([10, 27, 35]);

    color(Accent_color)
        translate([side * 25, -2, 34])
            rotate([0, side * 28, side * 12])
                rounded_bar(18, 3, 2.5);
}

module duck_face() {
    eye_z = Mood == "Thinking" ? 72 : 71;
    pupil_shift = Mood == "Thinking" ? 2.0 : 0;

    // Eyes and animated pupils.
    for (side = [-1, 1]) {
        color([1, 1, 1])
            translate([side * 7.2, -15.2, eye_z])
                rotate([90, 0, 0]) ellipsoid([10, 3.2, 11]);
        color([0.02, 0.025, 0.04])
            translate([side * 7.2 + pupil_shift, -17.0, eye_z + (Mood == "Ship it" ? 1 : 0)])
                sphere(d = 4.8);
        color(Accent_color)
            translate([side * 6.5 + pupil_shift, -18.8, eye_z + 1.2])
                sphere(d = 1.25);
    }

    // A smile-like open beak; its angle changes with mood.
    beak_angle = Mood == "Thinking" ? -8 : Mood == "Ship it" ? 8 : 2;
    color(beak_color())
        translate([0, -21.5, 62])
            rotate([beak_angle, 0, 0])
                ellipsoid([25, 17, 8]);

    color([0.42, 0.025, 0.04])
        translate([0, -29.1, 61.8])
            rotate([82 + beak_angle, 0, 0])
                ellipsoid([14, 2.2, 3.5]);
}

module crown() {
    color([1.00, 0.78, 0.08])
        translate([0, -1, 84])
            difference() {
                cylinder(h = 8, r1 = 9, r2 = 11, $fn = 6);
                translate([0, 0, 3]) cylinder(h = 7, r = 6.8, $fn = 6);
                for (angle = [0:60:300])
                    rotate([0, 0, angle])
                        translate([0, 8.5, 9])
                            rotate([45, 0, 0]) cube([5, 8, 8], center = true);
            }
    color(Accent_color)
        translate([0, -10.2, 89]) sphere(d = 3.5, $fn = 24);
}

module rainbow_tail() {
    for (i = [0:len(rainbow) - 1])
        color(rainbow[i])
            translate([(i - (len(rainbow) - 1) / 2) * 10, 9, 52 + abs(i - 2.5) * 2])
                rotate([-22, 0, (i - 2.5) * 8])
                    ellipsoid([8, 10, 29]);
}

module bug(scale_factor = 1, bug_color = [1.00, 0.12, 0.38]) {
    scale(scale_factor) {
        color(bug_color) {
            ellipsoid([8, 11, 6]);
            translate([0, -5, 0]) sphere(d = 5.5);
            for (side = [-1, 1], y = [-3, 0, 3])
                translate([side * 4, y, 0])
                    rotate([0, side * 65, 0]) cylinder(h = 7, d = 1.2);
        }
        color([0.04, 0.04, 0.06])
            for (side = [-1, 1])
                translate([side * 1.3, -7.2, 1.2]) sphere(d = 1.4);
    }
}

module caught_bug() {
    translate([0, -31, 65])
        rotate([75, 0, 0])
            bug(0.75);
}

module orbiting_bugs() {
    for (i = [0:Bug_count - 1]) {
        angle = 360 * i / Bug_count + 25;
        bug_height = 40 + 10 * sin(angle * 2);
        translate([42 * cos(angle), 33 * sin(angle), bug_height])
            rotate([70, 0, angle + 90])
                bug(0.45, rainbow[i % len(rainbow)]);
    }
}

module duck_skeleton() {
    color([1, 1, 1, 0.85]) {
        translate([0, 1, 34]) sphere(d = 7);
        translate([0, -1, 51]) cylinder(h = 17, d = 3);
        translate([0, -2, 67]) sphere(d = 6);
        for (side = [-1, 1])
            translate([0, 0, 45])
                rotate([0, side * 62, 0]) cylinder(h = 24, d = 2.4);
    }
}

module build_volume() {
    color([0.1, 0.8, 1.0, 0.12])
        difference() {
            translate([0, 0, 48]) cube([92, 92, 96], center = true);
            translate([0, 0, 48]) cube([89, 89, 93], center = true);
        }
}

module solid_duck() {
    duck_body_shell();
    duck_wing(-1);
    duck_wing(1);
    duck_face();
    caught_bug();
    orbiting_bugs();

    if (Rainbow_tail)
        rainbow_tail();
    if (Ship_it_crown || Mood == "Ship it")
        crown();
}

module debug_layers() {
    // # highlights the caught bug and emits energetic diagnostic rays.
    if (Debug_view == "Highlights")
        #union() {
            caught_bug();
            for (angle = [-35, 0, 35])
                translate([0, -32, 68])
                    rotate([angle, 0, 0]) cylinder(h = 9, d1 = 1.5, d2 = 0.2);
        }

    // % renders a translucent background shell without changing exported geometry.
    if (Debug_view == "Ghost shell")
        %scale([1.12, 1.12, 1.08]) duck_body_shell([0.1, 0.9, 1.0, 0.16]);

    if (Debug_view == "X-ray") {
        %solid_duck();
        #duck_skeleton();
    }

    if (Show_build_volume)
        %build_volume();
}

module rubber_duck_debugger() {
    scale(Duck_scale) {
        code_pedestal();
        if (Debug_view != "X-ray")
            solid_duck();
        debug_layers();
    }
}

// * disables the bug that tried to escape into production.
*translate([0, 0, 120]) bug(3, [1, 0, 0]);

// Root-operator recipe (opt-in because some host previewers suppress ! output):
// !rubber_duck_debugger();
rubber_duck_debugger();
