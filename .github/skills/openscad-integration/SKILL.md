---
name: openscad-integration
description: Create OpenSCAD files for complex 3D geometry in URDF, leveraging available libraries and automatic STL conversion
---

# OpenSCAD Integration for URDF Skill

This skill guides the creation and use of OpenSCAD files for custom 3D geometry in URDF robot descriptions.

## When to Use This Skill

Use OpenSCAD when:
- Basic URDF geometry (box, cylinder, sphere) is insufficient
- You need parametric, programmable 3D shapes
- You want to leverage existing OpenSCAD libraries
- You need maintainable, version-control-friendly geometry (vs binary STL)

## ⚠️ Important: Always Ask Permission

Before creating OpenSCAD files, ask the user:
- Creating .scad files adds complexity
- User may prefer different CAD workflow
- May require understanding OpenSCAD syntax

**Always ask**: "I can create an OpenSCAD file to generate this geometry. It will be automatically converted to STL. Would you like me to do that?"

## OpenSCAD Basics

### What is OpenSCAD?

OpenSCAD is a script-based 3D CAD modeler:
- Write code to describe geometry
- Parametric (easy to modify dimensions)
- Text-based (works great with version control)
- Rich ecosystem of libraries

### How the Extension Handles OpenSCAD

1. You create a `.scad` file in your project
2. Extension automatically converts it to `.stl` when saved
3. Reference the `.stl` file in your URDF
4. Preview updates automatically when `.scad` changes

**File watching**: The extension monitors `.scad` files and regenerates STL on save.

## Basic OpenSCAD Syntax

### Primitive Shapes

```scad
// Cube
cube([width, depth, height]);
cube([10, 20, 30]);  // 10mm × 20mm × 30mm box

// Sphere
sphere(radius);
sphere(r=15);  // 15mm radius sphere

// Cylinder
cylinder(h=height, r=radius);
cylinder(h=50, r=10);  // 50mm tall, 10mm radius

// Cylinder with different top/bottom radii (cone)
cylinder(h=30, r1=15, r2=5);
```

### Transformations

```scad
// Translate (move)
translate([x, y, z])
  cube([10, 10, 10]);

// Rotate (in degrees)
rotate([rx, ry, rz])
  cylinder(h=20, r=5);

// Scale
scale([sx, sy, sz])
  sphere(r=10);

// Combine transformations
translate([0, 0, 10])
  rotate([0, 90, 0])
    cylinder(h=30, r=5);
```

### Boolean Operations

```scad
// Union (combine shapes) - default behavior
union() {
  cube([20, 20, 20]);
  translate([10, 10, 10])
    sphere(r=12);
}

// Difference (subtract)
difference() {
  cube([30, 30, 30]);  // Main shape
  translate([15, 15, 0])  // Subtract this
    cylinder(h=30, r=8);
}

// Intersection (only overlapping parts)
intersection() {
  cube([20, 20, 20]);
  sphere(r=15);
}
```

### Modules (Functions)

```scad
// Define a reusable module
module bracket(width, height, thickness) {
  difference() {
    cube([width, height, thickness]);
    // Mounting holes
    translate([5, height/2, 0])
      cylinder(h=thickness, r=2);
    translate([width-5, height/2, 0])
      cylinder(h=thickness, r=2);
  }
}

// Use the module
bracket(50, 30, 5);
bracket(width=60, height=40, thickness=8);
```

## Common Patterns for Robotics

### Pattern 1: Wheel with Tread

```scad
// wheel_tread.scad
module wheel_with_tread(radius=50, width=30, tread_depth=3, tread_count=12) {
  difference() {
    // Main wheel body
    cylinder(h=width, r=radius, center=true);
    
    // Tread grooves
    for (i = [0:tread_count-1]) {
      rotate([0, 0, i * 360/tread_count])
        translate([radius - tread_depth/2, 0, 0])
          cube([tread_depth, 3, width+1], center=true);
    }
  }
}

// Generate wheel
wheel_with_tread(radius=100, width=40, tread_depth=5, tread_count=16);
```

**URDF reference:**
```xml
<link name="wheel">
  <visual>
    <geometry>
      <!-- Extension auto-converts wheel_tread.scad to wheel_tread.stl -->
      <mesh filename="package://my_robot/meshes/wheel_tread.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
</link>
```

### Pattern 2: Gripper Finger

```scad
// gripper_finger.scad
module gripper_finger(length=60, width=15, thickness=8) {
  difference() {
    // Main finger body
    hull() {
      cube([thickness, width, thickness]);
      translate([length-10, 0, 0])
        cube([10, width, thickness]);
    }
    
    // Grip texture (small bumps)
    for (i = [10:10:length-15]) {
      translate([i, width/2, thickness])
        sphere(r=2);
    }
  }
}

gripper_finger(length=70, width=18, thickness=10);
```

### Pattern 3: Sensor Housing

```scad
// sensor_housing.scad
module sensor_housing(sensor_diameter=20, wall_thickness=3) {
  difference() {
    // Outer shell
    cylinder(h=30, r=sensor_diameter/2 + wall_thickness);
    
    // Inner cavity for sensor
    translate([0, 0, wall_thickness])
      cylinder(h=25, r=sensor_diameter/2);
    
    // Mounting holes
    for (angle = [0:90:270]) {
      rotate([0, 0, angle])
        translate([sensor_diameter/2 + wall_thickness/2, 0, 15])
          rotate([90, 0, 0])
            cylinder(h=wall_thickness+2, r=2);
    }
  }
}

sensor_housing(sensor_diameter=25, wall_thickness=4);
```

### Pattern 4: Custom Bracket

```scad
// mounting_bracket.scad
module l_bracket(width=50, height=40, depth=30, thickness=5) {
  union() {
    // Vertical plate
    cube([thickness, depth, height]);
    
    // Horizontal plate
    cube([width, depth, thickness]);
    
    // Reinforcement gusset
    hull() {
      translate([0, depth/2, 0])
        cube([thickness, 1, thickness]);
      translate([0, depth/2, height-thickness])
        cube([thickness, 1, 1]);
    }
  }
}

l_bracket(width=60, height=50, depth=35, thickness=6);
```

## Using OpenSCAD Libraries

### Checking Available Libraries

Before writing custom code, check what's available:

1. **Run**: Command Palette → "URDF: Generate OpenSCAD Libraries Documentation"
2. **Review**: Generated markdown shows all available modules/functions
3. **Use**: Include or use the library in your .scad file

### Library Locations

The extension automatically loads libraries from:
- **Workspace root**: Any .scad files in your project
- **SCAD file directory**: Same directory as your .scad file (highest priority)
- **OS-specific defaults**:
  - Linux: `~/.local/share/OpenSCAD/libraries`
  - macOS: `~/Documents/OpenSCAD/libraries`
  - Windows: `%USERPROFILE%\Documents\OpenSCAD\libraries`
- **Custom paths**: Configured in `urdf-editor.OpenSCADLibraryPaths` setting

### Common Libraries

#### MCAD (Mechanical CAD)

```scad
include <MCAD/motors.scad>
use <MCAD/gears.scad>
use <MCAD/bearing.scad>

// Standard stepper motor
stepper_motor_mount(28);

// Spur gear
gear(number_of_teeth=20, 
     circular_pitch=5,
     gear_thickness=5,
     rim_thickness=5);
```

#### BOSL2 (Belfry OpenSCAD Library 2)

```scad
include <BOSL2/std.scad>

// Rounded box
cuboid([50, 30, 20], rounding=5);

// Threaded rod
threaded_rod(d=10, l=50, pitch=2);

// Grid of holes
grid_copies(spacing=20, n=3)
  cylinder(h=10, r=3);
```

### Include vs Use

```scad
// include: Makes everything available (variables, modules)
include <MCAD/motors.scad>

// use: Only makes modules available (not variables)
use <MCAD/gears.scad>
```

**Best practice**: Use `use` unless you need the library's variables.

## Integration with URDF

### File Organization

Recommended structure:
```
my_robot/
├── urdf/
│   └── robot.urdf.xacro
├── meshes/
│   ├── wheel.scad          ← OpenSCAD source
│   ├── wheel.stl           ← Auto-generated
│   ├── gripper.scad
│   └── gripper.stl
└── scad/                   ← Optional: keep .scad separate
    └── library.scad
```

### Workflow

1. **Create .scad file** in `meshes/` directory
2. **Save file** → Extension auto-generates `.stl`
3. **Reference .stl in URDF**:
```xml
<link name="custom_part">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/wheel.stl"/>
    </geometry>
  </visual>
</link>
```
4. **Preview URDF** → See the generated geometry
5. **Modify .scad** → Auto-regenerates STL on save

### Units and Scaling

**Important**: OpenSCAD typically uses millimeters, URDF uses meters.

```xml
<!-- Scale from mm to m (divide by 1000) -->
<mesh filename="package://my_robot/meshes/part.stl" 
      scale="0.001 0.001 0.001"/>
```

**Tip**: Document units in your .scad file:
```scad
// Units: millimeters
// This will be scaled to meters in URDF (scale="0.001 0.001 0.001")

module wheel(diameter=100) {  // 100mm = 0.1m
  cylinder(h=40, r=diameter/2);
}
```

## Advanced Techniques

### Parametric Design

Create families of related parts:
```scad
// parametric_wheel.scad
// Parameters can be overridden when including this file

wheel_diameter = 100;  // mm
wheel_width = 40;      // mm
hub_diameter = 20;     // mm
spoke_count = 5;

module parametric_wheel() {
  difference() {
    // Rim
    cylinder(h=wheel_width, r=wheel_diameter/2, center=true);
    
    // Hub cavity
    cylinder(h=wheel_width+2, r=hub_diameter/2, center=true);
    
    // Spokes (material removal between spokes)
    for (i = [0:spoke_count-1]) {
      rotate([0, 0, i * 360/spoke_count + 360/(2*spoke_count)])
        translate([hub_diameter/2 + (wheel_diameter-hub_diameter)/4, 0, 0])
          cube([wheel_diameter/2, 5, wheel_width+2], center=true);
    }
  }
}

parametric_wheel();
```

### Animation and Testing

Use animation to verify moving parts:
```scad
// Animate to test range of motion
$fn = 50;  // Resolution

module articulated_part(angle=0) {
  // Base
  cube([50, 30, 10]);
  
  // Moving part
  translate([25, 15, 10])
    rotate([angle, 0, 0])
      translate([0, 0, 20])
        cube([20, 10, 40], center=true);
}

// Use $t for animation (0 to 1)
articulated_part(angle=$t * 180 - 90);
```

### Resolution Control

```scad
// $fn controls circle resolution
$fn = 100;  // High quality (slower)
// $fn = 30;   // Medium (default-ish)
// $fn = 12;   // Low quality (faster)

cylinder(h=10, r=5);  // Uses $fn setting

// Override for specific shapes
cylinder(h=10, r=5, $fn=6);  // Hexagon
```

## Best Practices

### DO:
✅ Ask permission before creating OpenSCAD files
✅ Use descriptive module names and parameters
✅ Add comments explaining what the code does
✅ Define dimensions as variables/parameters
✅ Document units (mm vs m) in comments
✅ Check for available libraries before writing from scratch
✅ Test with preview after creating/modifying
✅ Keep .scad files in version control

### DON'T:
❌ Create overly complex geometry (affects preview performance)
❌ Forget to scale from mm to m in URDF
❌ Hardcode dimensions (use parameters instead)
❌ Create .scad files for simple shapes (use URDF primitives)
❌ Ignore available libraries (don't reinvent the wheel)
❌ Mix .scad logic with URDF structure

## Troubleshooting

### STL not generating?
- Check OpenSCAD syntax (look for errors in output)
- Verify file saved properly
- Check extension output channel for conversion errors

### Preview not updating?
- Save the .scad file again
- Refresh the URDF preview
- Check that .stl file was regenerated (timestamp)

### Library not found?
- Run "Generate OpenSCAD Libraries Documentation" to see available libraries
- Check library paths in settings: `urdf-editor.OpenSCADLibraryPaths`
- Verify library is in one of the standard locations

### Geometry looks wrong?
- Check units (mm in OpenSCAD, m in URDF)
- Verify scale attribute in URDF mesh tag
- Review $fn resolution setting
- Test .scad file in standalone OpenSCAD application

## Example Workflow

**User**: "I need a wheel with 12 spokes for my robot"

**Agent**: 
```
Basic cylinder geometry won't give you spokes. I can create an 
OpenSCAD file to generate a wheel with spokes. This will:
- Create a parametric wheel design
- Auto-generate STL on save
- Allow easy dimension changes

Would you like me to create this?
```

**After approval**:

```scad
// meshes/spoke_wheel.scad
// Units: millimeters (scale by 0.001 in URDF)

module spoke_wheel(diameter=100, width=30, hub=20, spokes=12, spoke_thickness=3) {
  difference() {
    union() {
      // Rim
      cylinder(h=width, r=diameter/2, center=true);
      
      // Hub
      cylinder(h=width, r=hub/2, center=true);
      
      // Spokes
      for (i = [0:spokes-1]) {
        rotate([0, 0, i * 360/spokes])
          translate([hub/2, 0, 0])
            cube([(diameter-hub)/2, spoke_thickness, width], center=true);
      }
    }
    
    // Center hole
    cylinder(h=width+2, r=hub/4, center=true);
  }
}

spoke_wheel(diameter=100, width=30, hub=20, spokes=12);
```

```xml
<!-- In URDF -->
<link name="wheel">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/spoke_wheel.stl" 
            scale="0.001 0.001 0.001"/>
    </geometry>
    <material name="wheel_color">
      <color rgba="0.2 0.2 0.2 1"/>
    </material>
  </visual>
</link>
```

## Summary

OpenSCAD integration provides:
- Parametric, maintainable 3D geometry
- Automatic STL generation
- Access to rich library ecosystem
- Version control friendly workflow
- Perfect for custom robot parts

**Remember**: Always ask before creating OpenSCAD files, check for existing libraries first, and test with preview!
