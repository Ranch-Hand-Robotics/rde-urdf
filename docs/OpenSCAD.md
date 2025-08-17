# OpenSCAD Preview and Language Support

OpenSCAD is a powerful tool for creating 3D models programmatically. This section covers how the URDF editor integrates OpenSCAD support, including file handling, previewing, and language features.

![OpenSCAD Preview](VSCodeOpenScad.png)

## OpenSCAD File Handling

The URDF editor can open and edit `.scad` files directly. When a `.scad` file is opened, the editor provides syntax highlighting and basic editing features.

## OpenSCAD Preview

The editor includes a preview feature for OpenSCAD files. When a `.scad` file is saved, the editor automatically converts it to STL format using the `openscad-wasm-prebuilt` module. The generated STL file is then displayed in the 3D viewer.

## OpenSCAD Language Features

The editor provides several language features for OpenSCAD:
- **Syntax Highlighting**: Basic syntax highlighting is available for OpenSCAD code.
- **Code Snippets**: Common OpenSCAD patterns are available as code snippets.
- **Error Reporting**: The editor reports syntax errors in OpenSCAD code.

## Acknowledgements

Special thanks to the OpenSCAD community for their contributions and support.
Thank you to [@lorenzowritescode](https://github.com/lorenzowritescode) for the the prebuilt OpenSCAD WASM module.