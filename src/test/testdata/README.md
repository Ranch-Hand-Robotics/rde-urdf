# URDF/Xacro Test Data

This directory contains test files for validating URDF and Xacro parsing functionality.

## Test Files Overview

### Simple URDF Files
- `simple_box.urdf` - Basic URDF with single link containing box geometry
- `two_links.urdf` - URDF with two links connected by a revolute joint

### Simple Xacro Files  
- `simple_xacro.xacro` - Basic xacro with properties and parameter substitution
- `macro_test.xacro` - Xacro demonstrating macro definitions and usage

### Mesh References
- `mesh_robot.urdf` - URDF referencing external STL and DAE mesh files
- `find_robot.xacro` - Xacro using $(find package) idiom for mesh paths

### Complex Multi-file Xacro
- `common_macros.xacro` - Shared macro definitions for wheels and arm segments
- `complex_robot.xacro` - Complex robot using includes and multiple macros

### Edge Cases and Error Conditions
- `edge_cases.xacro` - Tests missing packages, invalid URIs, and conditional content

### Gazebo Integration
- `gazebo_robot.xacro` - Robot with Gazebo-specific elements, sensors, and plugins

### Test Package Structure
- `test_robot/` - Mock ROS package with package.xml and meshes directory
- `robot_description/` - Mock package for testing $(find) idiom resolution

## Expected Test Scenarios

1. **Basic URDF Parsing** - Verify simple geometric shapes and materials render correctly
2. **Joint Mechanics** - Test different joint types (revolute, continuous, prismatic, fixed)
3. **Xacro Property Substitution** - Ensure variables and calculations work properly
4. **Macro Expansion** - Verify macro definitions and instantiation
5. **File Inclusion** - Test xacro:include functionality with relative paths
6. **Package Resolution** - Verify package:// URI and $(find) idiom resolution
7. **Error Handling** - Test behavior with missing files and invalid references
8. **Mesh Loading** - Verify STL and DAE file references resolve correctly
9. **Gazebo Elements** - Test handling of Gazebo-specific XML elements
10. **Complex Hierarchies** - Verify robots with multiple links and joints

## Usage in Tests

These files can be used to:
- Unit test the XacroParser integration
- Validate package resolution functionality  
- Test error handling for missing resources
- Benchmark parsing performance
- Verify UI rendering of different robot configurations

## Mesh File References

The test files reference the following mesh files (not included):
- `base.stl`, `base_collision.stl` - Robot base meshes
- `arm.dae` - Robotic arm mesh with materials
- `chassis.stl` - Vehicle chassis mesh
- `sensor_mount.dae` - Sensor mounting bracket
- `gripper.stl` - End effector gripper mesh

These files should be created or mock implementations provided for complete testing.
