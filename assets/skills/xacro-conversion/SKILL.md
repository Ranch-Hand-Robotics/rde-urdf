---
name: xacro-conversion
description: Convert URDF files to Xacro format with macros for reusable components like wheels, sensors, and repeated geometry
---

# URDF to Xacro Conversion Skill

This skill guides the process of converting URDF files to Xacro format and creating reusable macros for repeated geometry.

## When to Use This Skill

Use this skill when you notice:
- Repeated geometry in a URDF file (multiple wheels, legs, sensors, etc.)
- Similar links with only position/orientation differences
- Large URDF files that would benefit from parameterization
- User explicitly requests Xacro conversion

## ⚠️ Important: Always Ask Permission

Converting URDF to Xacro can be disruptive because:
- File extension changes from `.urdf` to `.xacro`
- Content structure changes significantly
- May affect existing launch files or references
- User may need to update package configuration

**Always ask**: "I can convert this to Xacro with macros for [components]. This will make it more maintainable but requires renaming the file. Would you like me to proceed?"

## Conversion Process

### Step 1: Identify Duplication

Look for repeated patterns:
- **Identical geometry**: Multiple wheels, legs, mounting brackets
- **Similar links**: Only differing in position, size, or name
- **Repeated joint patterns**: Same joint type with different parents/children

**Example of duplication:**
```xml
<!-- Four nearly identical wheel definitions -->
<link name="left_front_wheel">
  <visual>
    <geometry><cylinder radius="0.1" length="0.05"/></geometry>
  </visual>
</link>

<link name="right_front_wheel">
  <visual>
    <geometry><cylinder radius="0.1" length="0.05"/></geometry>
  </visual>
</link>
<!-- ... and two more identical wheels -->
```

### Step 2: Design the Macro

Identify what varies between instances:
- **Names** (prefix/suffix)
- **Positions** (x, y, z coordinates)
- **Orientations** (roll, pitch, yaw)
- **Sizes** (dimensions, radius, length)
- **Materials/colors**

**Macro design principles:**
- One macro per logical component type
- Clear parameter names
- Reasonable defaults
- Document what each parameter does

### Step 3: Create the Xacro File

**File structure:**
```xml
<?xml version="1.0"?>
<robot name="robot_name" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Properties (constants) -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_length" value="0.05"/>
  
  <!-- Macros -->
  <xacro:macro name="component_name" params="param1 param2 ...">
    <!-- Macro content -->
  </xacro:macro>
  
  <!-- Main robot structure -->
  <link name="base_link">
    <!-- ... -->
  </link>
  
  <!-- Use macros -->
  <xacro:component_name param1="value1" param2="value2"/>
  
</robot>
```

### Step 4: Implement the Macro

**Basic wheel macro example:**
```xml
<xacro:macro name="wheel" params="prefix reflect">
  <link name="${prefix}_wheel">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
      </geometry>
    </collision>
  </link>
  
  <joint name="base_to_${prefix}_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel"/>
    <origin xyz="0 ${reflect * wheel_offset_y} 0" rpy="0 1.5708 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</xacro:macro>
```

### Step 5: Use the Macro

```xml
<!-- Replace four wheel definitions with four macro calls -->
<xacro:wheel prefix="left_front" reflect="1"/>
<xacro:wheel prefix="right_front" reflect="-1"/>
<xacro:wheel prefix="left_rear" reflect="1"/>
<xacro:wheel prefix="right_rear" reflect="-1"/>
```

### Step 6: Rename and Verify

1. **Rename file**: `robot.urdf` → `robot.urdf.xacro` or `robot.xacro`
2. **Test preview**: Use "URDF: Preview" to verify the conversion
3. **Export if needed**: Use "URDF: Export" to generate expanded URDF for validation

## Common Macro Patterns

### Pattern 1: Wheel Macro

**Parameters:**
- `prefix` - Name prefix (left/right, front/rear)
- `x_pos`, `y_pos` - Position coordinates
- Optional: `reflect` - Mirror multiplier for symmetry

```xml
<xacro:macro name="wheel" params="prefix x_pos y_pos">
  <link name="${prefix}_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="base_to_${prefix}_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel"/>
    <origin xyz="${x_pos} ${y_pos} 0" rpy="0 1.5708 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</xacro:macro>

<!-- Usage -->
<xacro:wheel prefix="left_front" x_pos="0.3" y_pos="0.2"/>
<xacro:wheel prefix="right_front" x_pos="0.3" y_pos="-0.2"/>
```

### Pattern 2: Leg Macro (Quadruped)

**Parameters:**
- `prefix` - Leg identifier (fl, fr, rl, rr)
- `x_pos`, `y_pos` - Hip position
- `leg_length`, `foot_length` - Segment lengths

```xml
<xacro:macro name="leg" params="prefix x_pos y_pos">
  <!-- Hip link -->
  <link name="${prefix}_hip">
    <visual>
      <geometry><sphere radius="0.05"/></geometry>
    </visual>
  </link>
  
  <!-- Upper leg link -->
  <link name="${prefix}_upper_leg">
    <visual>
      <geometry><cylinder radius="0.02" length="0.3"/></geometry>
    </visual>
  </link>
  
  <!-- Lower leg link -->
  <link name="${prefix}_lower_leg">
    <visual>
      <geometry><cylinder radius="0.015" length="0.3"/></geometry>
    </visual>
  </link>
  
  <!-- Joints connecting segments -->
  <joint name="base_to_${prefix}_hip" type="revolute">
    <parent link="base_link"/>
    <child link="${prefix}_hip"/>
    <origin xyz="${x_pos} ${y_pos} 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>
  
  <joint name="${prefix}_hip_to_upper" type="revolute">
    <parent link="${prefix}_hip"/>
    <child link="${prefix}_upper_leg"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  
  <joint name="${prefix}_upper_to_lower" type="revolute">
    <parent link="${prefix}_upper_leg"/>
    <child link="${prefix}_lower_leg"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</xacro:macro>
```

### Pattern 3: Sensor Mounting Macro

**Parameters:**
- `name` - Sensor identifier
- `parent` - Parent link name
- `xyz`, `rpy` - Mounting position and orientation
- `mesh_file` - Optional mesh file path

```xml
<xacro:macro name="camera_mount" params="name parent x y z roll pitch yaw">
  <link name="${name}_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="camera_gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>
  
  <joint name="${parent}_to_${name}" type="fixed">
    <parent link="${parent}"/>
    <child link="${name}_link"/>
    <origin xyz="${x} ${y} ${z}" rpy="${roll} ${pitch} ${yaw}"/>
  </joint>
</xacro:macro>

<!-- Usage -->
<xacro:camera_mount name="front_camera" parent="base_link" 
                     x="0.3" y="0" z="0.1" 
                     roll="0" pitch="0" yaw="0"/>
```

## Advanced Xacro Features

### Properties (Constants)

Define once, use everywhere:
```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_length" value="0.05"/>
<xacro:property name="base_width" value="0.4"/>
<xacro:property name="pi" value="3.14159265"/>

<!-- Use in geometry -->
<cylinder radius="${wheel_radius}" length="${wheel_length}"/>
```

### Mathematical Expressions

Xacro supports math in `${}`:
```xml
<xacro:property name="wheel_offset" value="0.15"/>

<!-- Calculate positions -->
<origin xyz="0 ${wheel_offset + 0.05} 0"/>
<origin xyz="0 ${-wheel_offset} 0"/>
<origin xyz="0 0 ${wheel_radius / 2}"/>
```

### Conditional Blocks

```xml
<xacro:macro name="leg" params="prefix has_foot:=true">
  <link name="${prefix}_leg">
    <!-- ... -->
  </link>
  
  <xacro:if value="${has_foot}">
    <link name="${prefix}_foot">
      <!-- ... -->
    </link>
  </xacro:if>
</xacro:macro>
```

### Including Other Xacro Files

Split large robots into modular files:
```xml
<!-- main_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot name="modular_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Include macro libraries -->
  <xacro:include filename="$(find my_robot)/urdf/wheel_macros.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/sensor_macros.xacro"/>
  
  <!-- Use included macros -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:camera_mount name="front" parent="base_link"/>
</robot>
```

## Workflow Example

**User has this URDF:**
```xml
<robot name="quad_bot">
  <link name="base_link">...</link>
  
  <!-- Four identical wheels, only position differs -->
  <link name="lf_wheel"><visual><geometry><cylinder radius="0.08" length="0.04"/></geometry></visual></link>
  <joint name="base_to_lf" type="continuous">...</joint>
  
  <link name="rf_wheel"><visual><geometry><cylinder radius="0.08" length="0.04"/></geometry></visual></link>
  <joint name="base_to_rf" type="continuous">...</joint>
  
  <link name="lr_wheel"><visual><geometry><cylinder radius="0.08" length="0.04"/></geometry></visual></link>
  <joint name="base_to_lr" type="continuous">...</joint>
  
  <link name="rr_wheel"><visual><geometry><cylinder radius="0.08" length="0.04"/></geometry></visual></link>
  <joint name="base_to_rr" type="continuous">...</joint>
</robot>
```

**Agent response:**
```
I notice you have four identical wheels that differ only in position. 
This would be much cleaner as a Xacro macro. 

Converting to Xacro will:
- Reduce file from ~200 lines to ~80 lines
- Make wheel parameters easy to change (single definition)
- Rename file from .urdf to .xacro

Would you like me to convert this to Xacro?
```

**After user approval:**
```xml
<?xml version="1.0"?>
<robot name="quad_bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:property name="wheel_radius" value="0.08"/>
  <xacro:property name="wheel_length" value="0.04"/>
  
  <xacro:macro name="wheel" params="prefix x_reflect y_reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
        </geometry>
      </visual>
    </link>
    
    <joint name="base_to_${prefix}" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x_reflect * 0.3} ${y_reflect * 0.25} 0" rpy="0 1.5708 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>
  
  <link name="base_link">...</link>
  
  <xacro:wheel prefix="lf" x_reflect="1" y_reflect="1"/>
  <xacro:wheel prefix="rf" x_reflect="1" y_reflect="-1"/>
  <xacro:wheel prefix="lr" x_reflect="-1" y_reflect="1"/>
  <xacro:wheel prefix="rr" x_reflect="-1" y_reflect="-1"/>
</robot>
```

## Checklist for Conversion

- [ ] Identify all duplicated geometry
- [ ] Ask user permission to convert
- [ ] Design macro with clear parameters
- [ ] Add xacro namespace to robot tag
- [ ] Create properties for constants
- [ ] Implement macro(s)
- [ ] Replace duplicated code with macro calls
- [ ] Rename file to .xacro extension
- [ ] Test with preview command
- [ ] Verify with export command

## Common Pitfalls to Avoid

❌ **Don't:**
- Convert without asking user first
- Create overly complex macros with too many parameters
- Forget the xmlns:xacro namespace
- Leave unused properties or macros
- Mix tabs and spaces in indentation

✅ **Do:**
- Ask permission before converting
- Keep macros focused and simple
- Use descriptive parameter names
- Document what each macro does
- Test thoroughly with preview

## Extension Integration

- **Preview**: Xacro files are automatically processed and previewed
- **Export**: Generate expanded URDF to verify macro expansion
- **Validation**: Schema validates Xacro syntax
- **IntelliSense**: Auto-complete for Xacro elements and properties
