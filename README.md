# Robot Developer Extensions for URDF
This extension provides developer tooling for Unified Robot Description Format (URDF) and Xacro Editing and Preview. 

![URDF Preview](https://raw.githubusercontent.com/Ranch-Hand-Robotics/rde-urdf/refs/heads/main/docs/URDF_Preview.png)

## Features
- 3D preview
- Link Visualization
- Joint Visualization
- Color Customization
- Camera Controls
- Gizmos for exploring a URDF
- Syntax highlighting
- Code completion
- Collision Visualization
- Schema Validation
- AI Assisted Coding prompts for Github Copilot
- [Virtual Reality preview](https://ranchhandrobotics.com/rde-urdf/WebXRPreview.html) of the model you are editing.
- (New!) (Preview) [Supports OpenSCAD Rendering](https://ranchhandrobotics.com/rde-urdf/OpenSCAD.html) and syntax Highlighting for programatically creating robot parts.
- (New!) (Preview) [Exposes an MCP Server](https://ranchhandrobotics.com/rde-urdf/mcp.html) allowing AI to check its work visually. 
- Virtual Reality preview of the model you are editing.


## Coming Soon
- [Physics Emulation](https://github.com/ranchhandrobotics/vscode_urdf/issues/4)
- [Kinematics Visualization](https://github.com/ranchhandrobotics/vscode_urdf/issues/5)
- [External URDF / Xacro References](https://github.com/ranchhandrobotics/vscode_urdf/issues/6)

## Usage
1. Open a URDF, .xacro, or OpenSCAD file.
2. Right click on the file and select "Preview", or press `Ctrl+Shift+P` and select "Preview"

### OpenSCAD Library Configuration
The extension automatically loads OpenSCAD libraries from OS-specific default locations:
- **Windows**: `%USERPROFILE%\Documents\OpenSCAD\libraries`
- **Linux**: `$HOME/.local/share/OpenSCAD/libraries` 
- **macOS**: `$HOME/Documents/OpenSCAD/libraries`

To add custom library paths:
1. Open VS Code settings (`Ctrl+,`)
2. Search for "urdf-editor.OpenSCADLibraryPaths"
3. Add additional library directories (supports `${workspace}` variable)

Example settings.json:
```json
{
  "urdf-editor.OpenSCADLibraryPaths": [
    "${workspace}/scad_libs",
    "C:\\MyLibraries\\OpenSCAD",
    "/usr/local/share/openscad/libraries"
  ]
}
```

### OpenSCAD Documentation Generation
The extension can automatically generate documentation for your OpenSCAD libraries:

1. Open the command palette (`Ctrl+Shift+P`)
2. Run "URDF: Generate OpenSCAD Libraries Documentation"
3. Choose where to save the markdown file
4. The extension will scan all library paths and extract:
   - Header comments from library files
   - Module and function signatures
   - Parameter documentation
   - Usage examples

This documentation is also available to AI assistants via the Model Context Protocol (MCP) for intelligent code completion and suggestions.

### Model Context Protocol (MCP) Server
The extension includes an MCP server that provides AI assistants with powerful capabilities:

- **Screenshot Tools**: AI can capture and verify visual output of URDF/Xacro/OpenSCAD files
  - `take_screenshot`: Screenshot of currently active preview
  - `take_screenshot_by_filename`: Screenshot any file by path (opens preview if needed)
- **Library Documentation**: AI can access comprehensive OpenSCAD library information
  - `get_openscad_libraries`: Returns markdown documentation of all available libraries

The MCP server starts automatically when you open a preview and is accessible via HTTP on port 3005 (configurable).


## Support
If you encounter any issues with this extension, the following resources are provided:

### Github Issues
Bugs and feature requests are handled through [Github Issues in the Repository](https://github.com/Ranch-Hand-Robotics/rde-urdf/issues). 
If you find that you are hitting the same issue as someone else, please give a :+1: or comment on an existing issue.
Please provide as much details as possible, including an isolated reproduction of the issue or a pointer to an online repository.

### Discussions
[Github Discussions](https://github.com/orgs/Ranch-Hand-Robotics/discussions) are provided for community driven general guidance, walkthroughs, or support.

## Sponsor
If you find this extension useful, please consider [sponsoring Ranch Hand Robotics](https://github.com/sponsors/Ranch-Hand-Robotics) to help support the development of this extension and other open source projects.

## Acknowledgements
I was the maintainer of Microsoft's [Visual Studio Code ROS Extension](http://aka.ms/ros/vscode). It is no longer maintained, and I've been given permission to externalize the components. The URDF editor was built on work done in my private personal time, and not associated with Microsoft. This extension is a stand alone implementation and will be maintained moving forward.

This extension relies on the [Xacro-Parser](https://www.npmjs.com/package/xacro-parser) by [GKJohnson](https://github.com/gkjohnson) for stand alone Xacro parsing without requiring ROS.

This extension also uses the [openscad-wasm-prebuilt](https://www.npmjs.com/package/openscad-wasm-prebuilt) package for OpenSCAD processing, which is a prebuilt version of OpenSCAD for use in web applications. **Important**: This package is licensed under GPL-2.0-or-later.

The code for this extension unapologetically uses AI generated code.

## License
Some of the code in this extension is based on the [Visual Studio Code ROS Extension](http://aka.ms/ros/vscode) which is licensed under the MIT License. It also depends on Babylon ROS and Babylon Collada Loader by Polyhobbyist, both of which are MIT licensed.

**Important Licensing Note**: This extension includes the GPL-2.0 licensed openscad-wasm-prebuilt dependency. While the main extension code is MIT licensed, the combination creates a complex licensing situation. Please see:

- [LICENSE-COMPATIBILITY.md](LICENSE-COMPATIBILITY.md) - Comprehensive licensing guide and compliance checklist
- [THIRD_PARTY_NOTICES.txt](THIRD_PARTY_NOTICES.txt) - Full GPL-2.0 license text and attribution

When redistributing this extension, you must comply with both MIT and GPL-2.0 license requirements.

This extension is licensed under the MIT License, except where noted otherwise.

