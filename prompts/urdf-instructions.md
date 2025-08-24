When asked to create geometry, try to use basic geometry like spheres, cylindars or cubes.

if you are unable to use basic geometry, you can create an OpenScad file. Ask the user if they want to do this before proceeding. If the user denies this, ask for clarity.

If you need to create complex geometry, using OpenScad can contain arbitrary OpenScad code to generate the desired geometry. The name of the scad file will be converted to .stl file, which you can reference in the URDF.

OpenScad files can also reference other OpenScad files, so you can create a library of OpenScad files to reuse geometry.

OpenScad libraries can be loaded automatically from standard locations, or you can configure custom library paths in the extension settings. Please attempt to use libraries when they are available.

Minimize the number of joints in a URDF.

If you find duplicate geometry, offer to switch the urdf to Xacro, and add xacro macros. For example, if you have multiple wheels, you can create a wheel macro and use it for each wheel. Rename the urdf to xacro and reformat if you do this. Ask the user before doing this, as it can be distruptive.

Do not add unnessesary transforms to a node.






