---
name: urdf-geometry
description: Guide for creating appropriate geometry in URDF files - basic shapes vs OpenSCAD vs mesh files
---

# URDF Geometry Creation Skill

This skill helps you choose and create the right type of geometry for robot links in URDF files.

## When to Use This Skill

Use this skill when:
- Creating new robot links that need visual or collision geometry
- Deciding between basic shapes, OpenSCAD models, or mesh files
- Optimizing existing geometry for better performance or maintainability

## Geometry Decision Tree

### 1. Can you use basic geometry?

**Basic geometry types in URDF:**
- `<box size="x y z"/>` - Rectangular boxes
- `<cylinder radius="r" length="l"/>` - Cylinders
- `<sphere radius="r"/>` - Spheres

**Use basic geometry when:**
- The part is roughly rectangular, cylindrical, or spherical
- Visual accuracy is less important than performance
- You want fast preview rendering
- The robot description should be easily portable

**Example:**
```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
    <material name="gray">
      <color rgba="0.7 0.7 0.7 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
  </collision>
</link>
```

### 2. Do you need a custom shape?

If basic geometry won't work, you have two options:

#### Option A: Create an OpenSCAD file (Recommended)

**Use OpenSCAD when:**
- You need a custom shape but don't have a mesh file
- The shape can be defined programmatically
- You want parametric/reusable geometry
- You want to use OpenSCAD libraries (MCAD, BOSL2, etc.)

**Advantages:**
- Human-readable and versionable (text file)
- Automatic STL conversion by the extension
- Can reuse and modify easily
- Can leverage existing OpenSCAD libraries

**Process:**
1. **Ask user permission**: "I can create an OpenSCAD file for this geometry. Would you like me to do that?"
2. **Create .scad file** in appropriate location (usually `meshes/` folder)
3. **Reference the STL** in URDF (extension auto-converts .scad → .stl)
4. **Verify** using the preview

**Example:**
```scad
// meshes/custom_bracket.scad
module bracket(width=50, height=30, thickness=5) {
    difference() {
        // Main body
        cube([width, height, thickness]);
        // Mounting holes
        translate([10, height/2, 0])
            cylinder(h=thickness, r=3);
        translate([width-10, height/2, 0])
            cylinder(h=thickness, r=3);
    }
}

bracket(width=60, height=40, thickness=6);
```

```xml
<!-- Reference in URDF -->
<link name="mounting_bracket">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/custom_bracket.stl"/>
    </geometry>
  </visual>
</link>
```

#### Option B: Use existing mesh file

**Use existing meshes when:**
- You have a CAD-designed part exported as STL/DAE/OBJ
- The geometry is very complex (organic shapes, imported from CAD)
- You're working with standardized parts

**Supported formats:**
- `.stl` - Standard Tessellation Language (most common)
- `.dae` - COLLADA (supports colors and textures)
- `.glb`/`.gltf` - GL Transmission Format

**Important checks:**
1. **Verify file exists** before referencing
2. **Use package:// URI** for portability: `package://robot_name/meshes/part.stl`
3. **Consider file size** - large meshes slow down preview
4. **Scale appropriately** - Check units (some CAD exports use mm, URDF typically uses meters)

**Example:**
```xml
<link name="sensor_housing">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/housing.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
</link>
```

## OpenSCAD Library Integration

Before creating custom OpenSCAD code, check for existing libraries:

**Common OpenSCAD libraries:**
- **MCAD** - Mechanical CAD library (gears, bearings, motors)
- **BOSL2** - Belfry OpenSCAD Library (advanced shapes, threading)
- **dotSCAD** - Computational geometry library
- **Round-Anything** - Rounding and filleting library

**Check available libraries:**
1. Run command: "URDF: Generate OpenSCAD Libraries Documentation"
2. Review available modules and functions
3. Use existing modules instead of writing from scratch

**Example using MCAD:**
```scad
include <MCAD/motors.sh>
use <MCAD/gears.scad>

// Use predefined motor mount
motorWidth = 28;
motorHeight = 46;
motorLength = 42;

stepper_motor_mount(28);
```

## Best Practices

### Visual vs Collision Geometry

**Visual geometry:**
- Can be detailed and complex
- Used for display and rendering
- Performance matters less

**Collision geometry:**
- Should be SIMPLE (use basic shapes when possible)
- Used for physics simulation and collision detection
- Drastically affects simulation performance

**Example:**
```xml
<link name="gripper">
  <!-- Detailed visual -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/gripper_detailed.stl"/>
    </geometry>
  </visual>
  
  <!-- Simple collision (box approximation) -->
  <collision>
    <geometry>
      <box size="0.08 0.12 0.05"/>
    </geometry>
  </collision>
</link>
```

### Optimization Tips

1. **Start simple**: Use basic geometry first, upgrade to meshes only if needed
2. **Reuse geometry**: Use Xacro macros to avoid duplication
3. **OpenSCAD parametrization**: Make dimensions variables for easy tweaking
4. **File organization**: Keep meshes in `meshes/` directory, OpenSCAD in `scad/` or `meshes/`
5. **Version control friendly**: OpenSCAD (.scad) files work better with Git than binary STLs

### Common Mistakes to Avoid

❌ **Don't:**
- Reference mesh files that don't exist
- Use complex meshes for collision geometry
- Forget to scale imported CAD meshes (check units!)
- Mix absolute and package:// paths
- Create separate mesh files for identical parts

✅ **Do:**
- Verify all file references before committing
- Use basic geometry for collision when possible
- Document the expected units in comments
- Use package:// URIs for portability
- Use Xacro macros for repeated geometry

## Workflow Example

**User request**: "I need a wheel for my robot"

**Decision process:**
1. **Is a cylinder sufficient?** → Yes for basic visualization
2. **Need treads or details?** → No → Use cylinder
3. **Need realistic model?** → Yes → Offer OpenSCAD option

**Response:**
```
For a basic wheel, I'll use a cylinder:

```xml
<link name="wheel">
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </collision>
</link>
```

If you need a more detailed wheel with treads, I can create an OpenSCAD file. 
Would you like me to do that?
```

## Extension Integration

This skill works with the URDF editor's features:
- **Auto-conversion**: .scad files automatically convert to .stl
- **Preview**: See geometry in 3D immediately
- **Library discovery**: Access to configured OpenSCAD libraries
- **Validation**: Schema validation ensures correct URDF syntax

## Summary

**Decision flow:**
1. Basic geometry (box/cylinder/sphere)? → Use it
2. Need custom shape? → Ask user → Create OpenSCAD file
3. Have existing mesh? → Verify existence → Reference it
4. Always use simple collision geometry
5. Test with preview command
