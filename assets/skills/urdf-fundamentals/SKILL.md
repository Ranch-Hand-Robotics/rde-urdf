---
name: urdf-fundamentals
description: Core URDF/Xacro syntax, joint types, kinematic structures, and URDF validation for robot descriptions
---

# URDF Fundamentals Skill

This skill provides comprehensive guidance on creating, structuring, and validating URDF (Unified Robot Description Format) files for robot descriptions.

## When to Use This Skill

Use this skill when:
- Creating a new URDF from scratch
- Understanding URDF/Xacro syntax and structure
- Designing kinematic chains and joint hierarchies
- Adding links, joints, or coordinate frames
- Debugging URDF validation errors
- Implementing inertial properties and mass
- Understanding and using package paths and mesh references

## URDF File Structure

### Basic Robot Structure

Every URDF file has this basic structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  
  <!-- Define links (rigid bodies) -->
  <link name="base_link">
    <!-- Visual geometry for rendering -->
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <!-- Collision geometry for physics -->
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    
    <!-- Inertial properties for dynamics -->
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Define joints (connections between links) -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0.2 0" rpy="0 1.57 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <!-- Define other links -->
  <link name="wheel_link">
    <!-- ... geometry ... -->
  </link>
  
</robot>
```

### Key Elements

**`<link>`** - Represents a rigid body in the robot:
- `name` - Unique identifier
- Usually contains: `<visual>`, `<collision>`, `<inertial>` elements
- The first link is typically the "base_link" (root of kinematic chain)

**`<joint>`** - Represents a connection between two links:
- `name` - Unique identifier
- `type` - Joint type (revolute, continuous, prismatic, fixed, etc.)
- `<parent>` - Parent link name
- `<child>` - Child link name
- `<origin>` - Relative position and orientation
- `<axis>` - Direction and rotation/translation axis
- `<limit>` - Joint constraints (for movable joints)

**`<origin>`** - Specifies relative pose:
- `xyz` - Position (x, y, z in meters)
- `rpy` - Orientation (roll, pitch, yaw in radians)

## Links in Detail

### Visual Geometry

Used for display and rendering:

```xml
<link name="example_link">
  <visual>
    <!-- Geometry type and dimensions -->
    <geometry>
      <box size="x y z"/>
      <!-- or -->
      <cylinder radius="r" length="l"/>
      <!-- or -->
      <sphere radius="r"/>
      <!-- or -->
      <mesh filename="package://robot_name/meshes/part.stl" scale="0.001 0.001 0.001"/>
    </geometry>
    
    <!-- Optional: Position and orientation within link -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    
    <!-- Optional: Material coloring -->
    <material name="blue">
      <color rgba="0 0 1 1"/>  <!-- R G B Alpha (0-1) -->
    </material>
  </visual>
</link>
```

**Geometry types:**
- `<box size="x y z"/>` - Rectangular box in meters
- `<cylinder radius="r" length="l"/>` - Cylinder with radius and length in meters
- `<sphere radius="r"/>` - Sphere with radius in meters
- `<mesh filename="path" scale="sx sy sz"/>` - External 3D mesh file (STL, DAE, OBJ)

**Material colors (RGBA):**
- `rgba="1 0 0 1"` - Red
- `rgba="0 1 0 1"` - Green
- `rgba="0 0 1 1"` - Blue
- `rgba="1 1 1 1"` - White
- `rgba="0 0 0 1"` - Black
- Last value (alpha) is transparency: 1 = opaque, 0 = transparent

### Collision Geometry

Used for physics simulation and collision detection. Should typically be simpler than visual geometry:

```xml
<link name="complex_part">
  <visual>
    <!-- Detailed visual mesh -->
    <geometry>
      <mesh filename="package://robot/meshes/detailed_part.stl"/>
    </geometry>
  </visual>
  
  <collision>
    <!-- Simplified collision boxes -->
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
  </collision>
</link>
```

**Best practices:**
- Keep collision geometry simple for performance
- Use multiple collision elements to approximate complex shapes:

```xml
<link name="gripper">
  <collision>
    <origin xyz="0 0 0"/>
    <geometry><box size="0.1 0.2 0.05"/></geometry>
  </collision>
  
  <collision>
    <origin xyz="0.08 0.1 0"/>
    <geometry><box size="0.04 0.1 0.05"/></geometry>
  </collision>
</link>
```

### Inertial Properties

Define mass and rotational inertia for dynamics simulation:

```xml
<link name="base_link">
  <inertial>
    <!-- Mass in kilograms -->
    <mass value="5.0"/>
    
    <!-- Inertia tensor (moment of inertia) -->
    <!-- ixx, iyy, izz = rotational inertia about x, y, z axes -->
    <!-- ixy, ixz, iyz = products of inertia (usually 0 for symmetric objects) -->
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
    
    <!-- Optional: Center of mass offset from link origin -->
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </inertial>
</link>
```

**Computing inertia:**
- For a box: `I = (1/12) * m * (h^2 + d^2)` where h, d are dimensions
- For a cylinder: `I_x = I_y = (1/12) * m * (3*r^2 + h^2)`, `I_z = (1/2) * m * r^2`
- For a sphere: `I = (2/5) * m * r^2`

**Simplified approach:**
If exact inertia is unknown, use rough estimates:
```xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
</inertial>
```

## Joints in Detail

### Joint Types

#### 1. **Fixed Joint**
No movement - connects two links rigidly:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>
```

**Use for:**
- Sensors mounted on the robot
- Non-moving parts
- Rigid attachments

#### 2. **Revolute Joint**
Rotational movement with limits (like a door hinge):

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotate around Y axis -->
  <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
</joint>
```

**Parameters:**
- `<axis>` - Rotation axis (unit vector, typically one value = 1)
- `<limit>`:
  - `lower`, `upper` - Min/max angles in radians
  - `effort` - Maximum torque in Newton-meters
  - `velocity` - Maximum angular velocity in rad/s

**Common axes:**
- `xyz="1 0 0"` - Rotate around X (roll)
- `xyz="0 1 0"` - Rotate around Y (pitch)
- `xyz="0 0 1"` - Rotate around Z (yaw)

#### 3. **Continuous Joint**
Unlimited rotation (like a wheel):

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.2 0.15 0" rpy="0 1.57 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

**Use for:**
- Wheels that spin indefinitely
- Rotating shafts
- Continuous rotating joints

**Note:** No `<limit>` element needed

#### 4. **Prismatic Joint**
Linear sliding movement (like a piston):

```xml
<joint name="linear_actuator" type="prismatic">
  <parent link="base_link"/>
  <child link="piston_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Move along Z axis -->
  <limit lower="0" upper="0.2" effort="100" velocity="0.1"/>
</joint>
```

**Parameters:**
- `<axis>` - Direction of movement (unit vector)
- `<limit>`:
  - `lower`, `upper` - Min/max position in meters
  - `effort` - Maximum force in Newtons
  - `velocity` - Maximum linear velocity in m/s

#### 5. **Planar Joint**
Movement in a 2D plane (advanced, less common)

#### 6. **Floating Joint**
Free 6-DOF movement (for flying robots or underwater vehicles)

### Joint Limits and Effort

**Always specify limits for revolute joints:**

```xml
<!-- Limited rotation: safe joint -->
<joint name="elbow" type="revolute">
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="2.0" effort="30" velocity="1.5"/>
  <!-- Can rotate -2 to +2 radians (~-114 to +114 degrees) -->
  <!-- Max torque: 30 Nm, Max speed: 1.5 rad/s -->
</joint>
```

**If you forget limits:** Many simulators will refuse to load the file!

## Kinematic Structures

### Building a Kinematic Chain

A robot is organized as a tree of links connected by joints:

```
                  base_link (root)
                      |
                      | shoulder_joint (revolute)
                      |
                  arm_link
                      |
                      | elbow_joint (revolute)
                      |
                forearm_link
                      |
                      | wrist_joint (revolute)
                      |
                  hand_link
```

**URDF representation:**

```xml
<robot name="robot_arm">
  <!-- Root link -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.3 0.3 0.1"/></geometry>
    </visual>
  </link>
  
  <!-- First joint and child link -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.15"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>
  
  <link name="arm_link">
    <visual>
      <geometry><cylinder radius="0.05" length="0.4"/></geometry>
    </visual>
  </link>
  
  <!-- Continue chain... -->
</robot>
```

### Important Kinematic Rules

1. **Each link (except root) must have exactly ONE parent joint**
2. **A link can have multiple child joints** (branching is okay)
3. **No cycles allowed** - Structure must be a tree
4. **Root link** - Convention is to name it `base_link`
5. **Unique names** - Every link and joint must have a unique name

### Example: Quadruped Robot

```
            base_link (root)
         /      |      \      \
        /       |       \      \
  fl_hip  fr_hip  rl_hip  rr_hip
   /        /      /        /
fl_udeg  fr_uleg  rl_uleg  rr_uleg
 /        /        /        /
fl_leg  fr_leg   rl_leg   rr_leg
```

## Coordinate Frames and Transforms

### Understanding `<origin>`

The `xyz` and `rpy` in `<origin>` define the transform from parent to child:

```xml
<joint name="example" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  <!-- camera_link is positioned 0.1m forward, 0.2m up from base_link -->
  <!-- With no rotation -->
</joint>
```

### Rotations (rpy - Roll Pitch Yaw)

Three rotations applied in order: X (roll), Y (pitch), Z (yaw):

```xml
<!-- 90 degree pitch (rotate around Y axis) -->
<origin xyz="0 0 0" rpy="0 1.57 0"/>

<!-- 45 degree roll and yaw (rotate around X and Z) -->
<origin xyz="1 2 3" rpy="0.785 0 0.785"/>
```

**Converting degrees to radians:** `radians = degrees * π / 180`
- 90° = 1.57 rad
- 45° = 0.785 rad
- 180° = 3.14 rad

### Visual Origin vs Joint Origin

These are different:

```xml
<link name="camera">
  <visual>
    <!-- Position of visual geometry WITHIN the link -->
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry><box size="0.05 0.05 0.05"/></geometry>
  </visual>
</link>

<joint name="camera_mount" type="fixed">
  <!-- Position of camera_link RELATIVE TO parent (base_link) -->
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="camera"/>
</joint>
```

## Package and Mesh References

### Using ROS Packages

Mesh files and resources use package-relative paths:

```xml
<!-- Correct: package:// URI (portable across machines) -->
<mesh filename="package://my_robot/meshes/wheel.stl"/>

<!-- Wrong: Absolute path (not portable) -->
<mesh filename="/home/user/ros_ws/my_robot/meshes/wheel.stl"/>

<!-- Wrong: Relative path (fragile) -->
<mesh filename="meshes/wheel.stl"/>
```

### Package Discovery

The extension finds packages by:
1. Scanning workspace for `package.xml` files
2. Using ROS package commands if available
3. Building a package name → path mapping

**Best practice:** Always use `package://` URIs in your URDF files.

### Mesh File Paths

Typical project structure:

```
my_robot/
├── package.xml
├── urdf/
│   └── my_robot.urdf  (or .xacro)
└── meshes/
    ├── base.stl
    ├── wheel.stl
    └── gripper/
        ├── finger_left.stl
        └── finger_right.stl
```

**References in URDF:**

```xml
<mesh filename="package://my_robot/meshes/base.stl"/>
<mesh filename="package://my_robot/meshes/wheel.stl"/>
<mesh filename="package://my_robot/meshes/gripper/finger_left.stl"/>
```

### Mesh Scaling

Mesh units may differ from URDF (which uses meters):

```xml
<!-- STL exported in millimeters, convert to meters -->
<mesh filename="package://my_robot/meshes/part.stl" scale="0.001 0.001 0.001"/>

<!-- Scale by 50% -->
<mesh filename="package://my_robot/meshes/part.stl" scale="0.5 0.5 0.5"/>
```

### Creating Complex Meshes with OpenSCAD

For complex geometry that cannot be created with basic shapes (boxes, cylinders, spheres), use OpenSCAD:

**⚠️ IMPORTANT: Unit Mismatch**
- **OpenSCAD uses millimeters (mm)**
- **URDF uses meters (m)**
- When referencing the STL in URDF, you MUST scale by `0.001` to convert mm → m

**Workflow:**
1. Create a `.scad` file with your geometry logic in **millimeters** (e.g., `meshes/robot_foot.scad`)
2. The VS Code extension **automatically converts** `.scad` → `.stl` when saved
3. Reference the generated **`.stl` file** in your URDF/Xacro (not the `.scad`)
4. **Always apply `scale="0.001 0.001 0.001"`** to convert from mm to meters

**Example:**

Create `meshes/robot_foot.scad` (dimensions in **millimeters**):
```scad
// robot_foot.scad - Complex foot geometry
// All dimensions are in MILLIMETERS
module robot_foot(length=100, width=50, height=20) {
  difference() {
    // Main foot body: 100mm × 50mm × 20mm
    cube([length, width, height]);
    
    // Mounting holes: 5mm radius
    translate([20, width/2, 0])
      cylinder(h=height, r=5);
    translate([length-20, width/2, 0])
      cylinder(h=height, r=5);
  }
}

// Generate foot with dimensions in mm
robot_foot(length=120, width=60, height=25);
```

Reference in URDF (with **0.001 scale** to convert mm → m):
```xml
<link name="foot">
  <visual>
    <geometry>
      <!-- 
        Reference the generated STL file (not the .scad).
        Scale 0.001 converts from OpenSCAD millimeters to URDF meters.
        The foot will be 0.12m × 0.06m × 0.025m in the robot.
      -->
      <mesh filename="package://my_robot/meshes/robot_foot.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <mesh filename="package://my_robot/meshes/robot_foot.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </collision>
</link>
```

**Important notes:**
- **Always use `scale="0.001 0.001 0.001"`** when referencing OpenSCAD-generated STL files
- OpenSCAD source (`.scad`) stays in millimeters - this is standard CAD convention
- Reference the `.stl` file in URDF, never the `.scad`
- The `.scad` file is the source; the `.stl` is what renders
- When you save the `.scad` file, the extension automatically regenerates the `.stl`
- You can see a live preview of the geometry using the preview command
- For parametric geometry reuse, create OpenSCAD modules that other `.scad` files can reference

## URDF Validation and Best Practices

### Common Errors

**Error: "Duplicate link/joint name"**
- Every link and joint name must be unique
- Check for typos or copy-paste mistakes

**Error: "Parent link not found"**
```xml
<!-- Wrong: Parent link doesn't exist -->
<joint name="wheel" type="continuous">
  <parent link="baes_link"/>  <!-- Typo! -->
  <child link="wheel"/>
</joint>
```

**Error: "Joint limits missing"**
```xml
<!-- Wrong: revolute joint without limits -->
<joint name="arm" type="revolute">
  <axis xyz="0 1 0"/>
  <!-- Missing <limit> tag -->
</joint>

<!-- Correct: Always specify limits -->
<joint name="arm" type="revolute">
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
</joint>
```

**Error: "Mesh file not found"**
- Verify mesh file exists
- Check spelling and path
- Ensure package name is correct

### Validation Workflow

1. **Syntax check** - XML structure is valid
2. **Preview** - Use "URDF: Preview" to visualize
3. **Take screenshot** - Verify geometry matches expectations
4. **Check kinematic tree** - Ensure links and joints form valid tree
5. **Inspect console** - Look for warning messages

### Naming Conventions

Follow ROS naming standards:

```xml
<!-- ✓ Good: lowercase with underscores -->
<link name="base_link"/>
<link name="left_front_wheel"/>
<joint name="base_to_wheel"/>
<joint name="shoulder_joint"/>

<!-- ✗ Bad: CamelCase or mixed case -->
<link name="BaseLink"/>
<link name="leftFrontWheel"/>
<joint name="baseToWheel"/>
```

### Comments and Documentation

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Root link representing the robot base -->
  <link name="base_link">
    <visual>
      <geometry>
        <!-- 50cm wide, 30cm deep, 10cm tall -->
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="base_color">
        <color rgba="0.8 0.8 0.8 1"/>  <!-- Light gray -->
      </material>
    </visual>
    
    <!-- Collision geometry approximates base as simple box -->
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
  </link>
  
  <!-- Front-left wheel connection -->
  <joint name="base_to_fl_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="fl_wheel"/>
    <!-- Position: 0.15m forward, 0.15m left, 0.05m down from base center -->
    <origin xyz="0.15 0.15 -0.05" rpy="0 1.5708 0"/>
    <!-- Wheel rotates around Y axis (left-right) -->
    <axis xyz="0 1 0"/>
  </joint>
  
  <link name="fl_wheel">
    <visual>
      <geometry>
        <!-- 10cm diameter wheel, 5cm wide -->
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
  </link>
  
</robot>
```

## Using URDF in the VS Code Extension

### Preview Commands

- **Right-click** URDF file → "URDF: Preview" to open 3D viewer
- **Right-click** URDF file → "URDF: Take Screenshot" for documentation
- **Right-click** Xacro file → "URDF: Export" to generate expanded URDF

### Validation Features

- XML schema automatically validates syntax
- Hover over elements for documentation
- Go to Definition (F12) jumps to referenced links/joints
- Diagnostics panel shows errors and warnings

### Mesh File Viewer

The extension includes a custom 3D viewer for mesh files:
- Opens `.stl`, `.dae`, `.glb`, `.gltf` files in 3D
- Supports interactive rotation, zoom, pan
- Displays mesh statistics and properties

## Quick Reference: Joint Type Selection

| Joint Type | Movement | Use Case |
|-----------|----------|----------|
| **Fixed** | None | Sensors, non-moving attachments |
| **Revolute** | Rotational (limited) | Robot arms, articulated segments |
| **Continuous** | Rotational (unlimited) | Wheels, rotating shafts |
| **Prismatic** | Linear (limited) | Actuators, sliding mechanisms |
| **Planar** | 2D planar | Advanced wheeled robots |
| **Floating** | Free 6-DOF | Flying/underwater robots |

## Best Practices Summary

✅ **DO:**
1. Start with `base_link` as the root
2. Use `package://` URIs for mesh files
3. Specify joint limits for revolute joints
4. Use simple collision geometry
5. Include inertial properties for dynamics
6. Use meaningful link and joint names
7. Take screenshots to verify geometry
8. Add XML comments explaining design choices
9. Validate in the extension's preview
10. Use Xacro macros for repeated structures

❌ **DON'T:**
1. Use absolute file paths
2. Forget joint limits
3. Make collision geometry too complex
4. Reference non-existent mesh files
5. Create cycles in the kinematic tree
6. Use inconsistent naming conventions
7. Skip inertial properties if doing simulation
8. Make geometry without visualizing it first
9. Assume mesh scales are in meters
10. Leave namespacing issues unresolved

