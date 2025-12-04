# Robot Development Extensions for URDF
This Visual Studio Code extension helps you buid Universal Robot Description Format (URDF) files for your robot. This extension does not require ROS to be installed in order to edit and visualize URDF files.

![URDF Preview](https://raw.githubusercontent.com/Ranch-Hand-Robotics/rde-urdf/refs/heads/main/docs/URDF_Preview.png)

## Features

### Core Editing & Visualization
- **3D Preview**: Real-time rendering with BabylonJS engine
- **Link & Joint Visualization**: Visual representation of robot structure with axis and rotation parameters
- **Collision Geometries**: Toggle between visual and collision geometry display
- **Joint Explosion**: "Explode" joints to see internal structure
- **Interactive Controls**: Camera controls, rotation gizmos, and axis visualization
- **Export to Isaac Simulation**: Direct export for NVIDIA Isaac Sim

### IntelliSense & Navigation
- **Go to Definition (F12)**:
  - Jump to URDF links, joints, xacro macros, and properties
  - Navigate to OpenSCAD modules and functions across files
  - Resolve package paths and mesh file references
  - Cross-file symbol resolution via includes
- **Smart Hover Documentation**:
  - URDF/Xacro elements with full XML signatures and inline attributes
  - OpenSCAD modules/functions with parameter descriptions from comments
  - Built-in element documentation with types, defaults, and units
  - Source file location for imported definitions
- **Code Completion**:
  - URDF/Xacro element snippets with intelligent placeholders
  - OpenSCAD built-in functions and transformations
  - Xacro property reference completion (`${}`)
  - Contextual suggestions based on cursor position
- **Schema Validation**: Real-time XML schema validation

### OpenSCAD Support
- [OpenSCAD Rendering](https://ranchhandrobotics.com/rde-urdf/OpenSCAD.html): Full support for programmatically creating robot parts
- **Integrated Editing**: Syntax highlighting and IntelliSense
- **Live Preview**: Automatic STL conversion and 3D rendering
- **Library Management**: Automatic discovery from workspace and system paths
- **Documentation Generation**: Extract module/function docs from comments

### AI-Assisted Development
- **GitHub Copilot Integration**: Custom prompts for URDF/Xacro/OpenSCAD
- [MCP Server](https://ranchhandrobotics.com/rde-urdf/mcp.html): Model Context Protocol for AI visual verification
  - Screenshot capture of 3D previews
  - OpenSCAD library documentation access
  - Visual feedback loop for code generation
- **Vibe Coding**: AI can see rendered output and suggest improvements

### Virtual Reality
- [WebXR Preview](https://ranchhandrobotics.com/rde-urdf/WebXRPreview.html): Immersive VR viewing and interactive exploration

## Coming soon
- [Physics Emulation](https://github.com/ranchhandrobotics/vscode_urdf/issues/4)
- [Kinematics Visualization](https://github.com/ranchhandrobotics/vscode_urdf/issues/5)
- [External URDF / Xacro References](https://github.com/ranchhandrobotics/vscode_urdf/issues/6)

## Quick Start
1. Open a URDF, .xacro, or OpenSCAD file
2. Right-click and select **"Preview"** or press `Ctrl+Shift+P` → "URDF: Preview"
3. Use **F12** (Go to Definition) on any link, joint, or module name to navigate
4. Hover over elements to see inline documentation with parameters

### Navigation Tips
- **F12** on link/joint names → Jump to definition
- **F12** on xacro macro calls → Jump to macro definition  
- **F12** on OpenSCAD module calls → Jump to module (even in libraries)
- **F12** on file paths (package://, mesh files) → Open the referenced file
- **Hover** on elements → See full signature with parameters and types
- **Ctrl+Space** → Trigger IntelliSense completion

### Configuration
See the [Configuration Guide](Configuration.md) for detailed setup options including ROS package paths, OpenSCAD libraries, and visual customization.




## Acknowledgements
This extension is rebranded and re-released by Ranch Hand Robotics, owned by the maintainer of the [ms-iot VSCode ROS Extension](https://github.com/ms-iot/vscode-ros) with permission from Microsoft. The source extension was split into 3 parts - [ROS 1](https://ranchhandrobotics.github.io/rde-ros-1/), [ROS 2](https://ranchhandrobotics.github.io/rde-ros-2/) and a [URDF editor](https://ranchhandrobotics.github.io/rde-urdf/).
