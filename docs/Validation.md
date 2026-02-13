# URDF/Xacro Validation

The URDF Editor extension includes comprehensive real-time validation for URDF and Xacro files. Errors and warnings are highlighted inline as you type, making it easier to catch and fix issues early in the development process.

## Features

### XML Syntax Validation
The validator checks for proper XML syntax including:
- Well-formed XML structure
- Properly closed tags
- Valid attribute syntax
- Character encoding issues

**Example errors detected:**
- Unclosed tags: `<box size="1 1 1"` (missing `/>`)
- Invalid characters in tag names
- Mismatched opening/closing tags

### URDF Structure Validation

#### Required Elements
The validator ensures your URDF follows the specification:
- Root `<robot>` element must exist
- Robot must have a `name` attribute
- All links must have a `name` attribute
- All joints must have a `name` attribute

**Example:**
```xml
<!-- ERROR: Missing name attribute -->
<robot>
  <link name="base_link">...</link>
</robot>

<!-- CORRECT -->
<robot name="my_robot">
  <link name="base_link">...</link>
</robot>
```

#### Joint Validation
Validates joint definitions including:
- Joint type must be one of: `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`
- Parent and child links must be defined (or be Xacro expressions)

**Example:**
```xml
<!-- ERROR: Invalid joint type -->
<joint name="my_joint" type="invalid_type">
  <parent link="base_link"/>
  <child link="end_effector"/>
</joint>

<!-- CORRECT -->
<joint name="my_joint" type="fixed">
  <parent link="base_link"/>
  <child link="end_effector"/>
</joint>
```

### Geometry Validation

#### Box Geometry
- Must have a `size` attribute
- Size must be three numbers (x y z dimensions)

**Example:**
```xml
<!-- ERROR: Wrong number of dimensions -->
<box size="1 1"/>

<!-- CORRECT -->
<box size="1 1 1"/>
```

#### Cylinder Geometry
- Must have `radius` and `length` attributes
- Both must be numeric values

**Example:**
```xml
<!-- ERROR: Missing radius -->
<cylinder length="1.0"/>

<!-- CORRECT -->
<cylinder radius="0.5" length="1.0"/>
```

#### Sphere Geometry
- Must have a `radius` attribute

**Example:**
```xml
<!-- ERROR: Missing radius -->
<sphere/>

<!-- CORRECT -->
<sphere radius="0.5"/>
```

#### Mesh Geometry
- Must have a `filename` attribute
- Filename should point to a mesh file (.stl, .dae, etc.)

**Example:**
```xml
<!-- ERROR: Missing filename -->
<mesh/>

<!-- CORRECT -->
<mesh filename="package://my_robot/meshes/base.stl"/>
```

### Link Reference Validation
The validator checks that all link references in joints point to defined links:
- Parent link must exist
- Child link must exist

**Example:**
```xml
<robot name="test">
  <link name="base_link">...</link>
  
  <!-- WARNING: undefined_link not found -->
  <joint name="test_joint" type="fixed">
    <parent link="base_link"/>
    <child link="undefined_link"/>
  </joint>
</robot>
```

### Character Set Validation
Ensures your files use valid UTF-8 encoding:
- Detects invalid control characters
- Warns about misplaced Byte Order Marks (BOM)
- Helps prevent encoding-related issues

## Xacro-Aware Validation

The validator recognizes Xacro syntax and won't flag these as errors:
- Property references: `${property_name}`
- ROS package paths: `$(find package_name)`
- Xacro expressions in attributes
- Macro-generated content

**Example:**
```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:property name="width" value="1.0"/>
  
  <!-- These Xacro expressions are recognized and validated correctly -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${width} ${width} ${width}"/>
      </geometry>
    </visual>
  </link>
  
  <!-- Package references are also recognized -->
  <link name="mesh_link">
    <visual>
      <geometry>
        <mesh filename="package://my_robot/meshes/part.stl"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Diagnostic Severity Levels

### Errors (Red Squiggles)
Critical issues that will prevent the URDF from working:
- XML syntax errors
- Missing required attributes
- Invalid geometry specifications
- Invalid joint types
- Malformed elements

### Warnings (Yellow Squiggles)
Potential issues that may cause problems:
- Undefined link references (unless using Xacro)
- Character encoding issues
- Deprecated patterns

## Using the Validator

The validator runs automatically when you:
1. Open a `.urdf` or `.xacro` file
2. Edit the file (validation updates as you type)
3. Save the file

### Viewing Diagnostics

**Problems Panel:**
1. Open the Problems panel: `View` → `Problems` or `Ctrl+Shift+M`
2. See all validation errors and warnings across your workspace
3. Click on any problem to jump to that location in your file

**Inline Indicators:**
- Squiggly underlines appear directly in the editor
- Hover over underlined text to see the error message
- Red = Error, Yellow = Warning

**Quick Fixes:**
Many validation errors include helpful messages explaining:
- What the problem is
- What the correct format should be
- Examples of valid syntax

## Best Practices

1. **Fix errors as you type**: The real-time validation helps catch issues immediately
2. **Use the Problems panel**: See all issues across multiple files at once
3. **Check warnings**: Even if not critical, warnings can indicate potential issues
4. **Test with preview**: Use the 3D preview to verify your URDF renders correctly
5. **Validate before sharing**: Ensure your URDF has no errors before committing

## Troubleshooting

### False Positives with Xacro
If the validator flags valid Xacro expressions:
- Ensure your file includes the Xacro namespace: `xmlns:xacro="http://www.ros.org/wiki/xacro"`
- Check that property/macro names are correctly referenced
- Verify expressions use proper `${}` or `$()` syntax

### Missing Link Warnings
If you get warnings about missing links that are actually defined:
- Check for typos in link names
- Ensure the link definition appears before it's referenced
- For macro-generated links, the validator may not detect them (this is expected)

### Character Encoding Issues
If you see character encoding warnings:
- Save your file as UTF-8 encoding
- Remove any Byte Order Marks (BOM) from the file
- Check for invisible control characters

## Additional Resources

- [URDF Specification](http://wiki.ros.org/urdf)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [XML Syntax Reference](https://www.w3.org/TR/xml/)

For more help, see our [Support Guide](Support.md) or file an issue on [GitHub](https://github.com/Ranch-Hand-Robotics/rde-urdf/issues).
