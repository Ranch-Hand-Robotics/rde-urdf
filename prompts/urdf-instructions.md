# URDF/Xacro/OpenSCAD Coding Guidelines

## MCP Server Usage - CRITICAL

**ALWAYS use the MCP server tools when working with URDF, Xacro, or OpenSCAD files. This is MANDATORY, not optional.**

### Required MCP Tool Usage:

1. **Before declaring ANY OpenSCAD work complete:**
   - MUST call `validate_openscad` with the filename to check for compilation errors
   - MUST fix all errors returned before proceeding
   - MUST re-validate after fixes
   - DO NOT claim completion until validation passes with zero errors

2. **After making changes to URDF/Xacro/OpenSCAD files:**
   - MUST call `take_screenshot` or `take_screenshot_by_filename` to verify visual output
   - MUST compare the screenshot with user requirements
   - MUST iterate if the visual output doesn't match expectations

3. **When working with OpenSCAD libraries:**
   - SHOULD call `get_openscad_libraries` to see available modules and functions
   - Use existing library functions instead of reinventing code

### Example Workflow:

```
1. Create/edit OpenSCAD file
2. Call validate_openscad → Fix errors if any
3. Call take_screenshot → Verify visual output
4. If issues found → Go back to step 1
5. Only then declare work complete
```

**FAILURE TO USE THESE TOOLS WILL RESULT IN BROKEN CODE BEING DELIVERED.**

## Geometry Guidelines

When asked to create geometry, try to use basic geometry like spheres, cylinders or cubes.

If you are unable to use basic geometry, you can create an OpenSCAD file. Ask the user if they want to do this before proceeding. If the user denies this, ask for clarity.

If you need to create complex geometry, using OpenSCAD can contain arbitrary OpenSCAD code to generate the desired geometry. The name of the scad file will be converted to .stl file, which you can reference in the URDF.

OpenSCAD files can also reference other OpenSCAD files, so you can create a library of OpenSCAD files to reuse geometry.

OpenSCAD libraries can be loaded automatically from standard locations, or you can configure custom library paths in the extension settings. Please attempt to use libraries when they are available.

## URDF Best Practices

Minimize the number of joints in a URDF.

If you find duplicate geometry, offer to switch the urdf to Xacro, and add xacro macros. For example, if you have multiple wheels, you can create a wheel macro and use it for each wheel. Rename the urdf to xacro and reformat if you do this. Ask the user before doing this, as it can be disruptive.

Do not add unnecessary transforms to a node.


