# URDF Editor VS Code Extension - AI Coding Instructions

## Project Overview
This is a VS Code extension for editing, previewing, and validating URDF (Unified Robot Description Format) and Xacro files. The extension provides 3D visualization using BabylonJS and supports WebXR preview for robotics development.

## Architecture
- **Extension Entry**: `src/extension.ts` - Main activation point, registers commands and providers
- **Preview System**: `src/previewManager.ts` + `src/preview.ts` - Manages URDF/Xacro/OpenSCAD 3D preview webviews
- **OpenSCAD Processing**: `src/openscad.ts` - OpenSCAD conversion utilities and library management
- **3D Viewer**: `src/3DViewerProvider.ts` + `src/3DViewerDocument.ts` - Custom editor for STL/DAE mesh files
- **Webview Frontend**: `src/webview/webview.ts` - BabylonJS-based 3D rendering in webview
- **URDF Processing**: `src/utils.ts` - Core Xacro parsing and package resolution logic

## Key Patterns

### Xacro Processing Pipeline
The extension uses a sophisticated pipeline for processing Xacro files:
1. `XacroParser` from `xacro-parser` npm package handles macro expansion
2. Custom `getFileContents` function resolves `$(find package_name)` idioms during parsing
3. Post-processing converts `package://` URIs to webview URIs for mesh loading
4. Missing packages are tracked and reported to user

Example from `utils.ts`:
```typescript
// Convert $(find package) to package:// format before parsing
export function convertFindToPackageUri(text: string): string {
  const findPattern = /\$\(find\s+([a-zA-Z0-9_-]+)\)/g;
  return text.replace(findPattern, (match, packageName) => {
    return `package://${packageName}`;
  });
}
```

### Package Resolution
Packages are discovered by scanning workspace for `package.xml` files. The `getPackages()` function in `utils.ts` builds a map of package names to filesystem paths, enabling resolution of ROS package references.

### OpenSCAD Processing
OpenSCAD (.scad) files are processed using dedicated utilities in `src/openscad.ts`:
1. File is detected as OpenSCAD by extension
2. OpenSCAD libraries are loaded from OS-specific and user-configured paths
3. `openscad-wasm-prebuilt` module converts .scad code to STL format with library support
4. Generated STL is written to the same directory as the .scad file
5. Existing STL viewer renders the converted file through webview
6. File watching triggers re-conversion on .scad file changes

#### Library Support
OpenSCAD library loading supports:
- **OS-specific default paths**:
  - Windows: `%USERPROFILE%\Documents\OpenSCAD\libraries`
  - Linux: `$HOME/.local/share/OpenSCAD/libraries`
  - macOS: `$HOME/Documents/OpenSCAD/libraries`
- **User-configured paths**: Via `urdf-editor.OpenSCADLibraryPaths` setting
- **Workspace variables**: `${workspace}` resolves to workspace root
- **Automatic discovery**: Only existing directories are included
- **Recursive loading**: Subdirectories and files (.scad, .stl, .dxf) loaded into virtual filesystem

#### Library Documentation
The extension can generate comprehensive documentation of available OpenSCAD libraries:
- **Command**: `urdf-editor.generateOpenSCADDocs` - Generate markdown documentation
- **MCP Tool**: `get_openscad_libraries` - Expose library info to AI assistants
- **Extraction**: Header comments, module signatures, function parameters, and inline documentation
- **Format**: Structured markdown suitable for AI consumption

Example from `src/openscad.ts`:
```typescript
// Load libraries and convert OpenSCAD to STL
const libraryPaths = await getAllOpenSCADLibraryPaths(workspaceRoot);
await loadLibraryFiles(instance, libraryPaths, this._trace);
instance.FS.writeFile('/input.scad', scadText);
instance.callMain(['-L', '/libraries', '-o', '/output.stl', '/input.scad']);

// Generate documentation for MCP
const documentation = await generateOpenSCADLibrariesDocumentation(workspaceRoot);
const markdown = convertLibrariesDocumentationToMarkdown(documentation);
````
```

### OpenSCAD Processing
OpenSCAD (.scad) files are processed in `3DViewerDocument.ts`:
1. File content is read from the filesystem
2. `openscad-wasm` module converts .scad code to STL format
3. Generated STL is written to a temporary file
4. Existing STL viewer renders the converted file
5. File watching triggers re-conversion on .scad file changes

### Webview Communication
Bidirectional communication between extension and webview uses message passing:
- Extension → Webview: URDF content, color settings, file paths
- Webview → Extension: Ready state, errors, trace messages

Commands sent to webview:
- `urdf`: Send processed URDF content for rendering
- `colors`: Update visual appearance settings
- `view3DFile`: Load individual 3D mesh file
- `previewFile`: Set preview file path

### Configuration Integration
Extension automatically configures GitHub Copilot to use domain-specific instructions:
```typescript
// Force workspace to use instruction files
vscode.workspace.getConfiguration().update('github.copilot.chat.codeGeneration.useInstructionFiles', true);
// Add URDF-specific prompts
instructions.push({ file: path.join(context.extensionPath, 'prompts/urdf-instructions.md') });
```

## Build & Development

### Build Commands
- `npm run watch` - Development build with file watching
- `npm run compile-tests` - Compile test files
- `npm run package` - Production build for packaging
- `npm run vsix` - Create .vsix package file

### Webpack Configuration
Dual webpack configs in `webpack.config.js`:
- **Extension bundle**: Node.js target for VS Code extension host
- **Webview bundle**: Web target for browser-based 3D viewer

Important: Assets (schemas, snippets, prompts) are copied via `CopyWebpackPlugin` to `dist/` folder.

### Testing
Tests are in `src/test/` with sample URDF/Xacro files in `src/test/testdata/`. Run via `npm run test` which compiles and executes using `@vscode/test-electron`.

## Dependencies & External Integration

### Key Dependencies
- `xacro-parser`: Standalone Xacro processing without ROS
- `babylonjs`: 3D rendering engine
- `@polyhobbyist/babylon_ros`: ROS-specific BabylonJS components
- `xmldom` + `jsdom`: XML parsing for URDF/Xacro content
- `openscad-wasm`: WebAssembly OpenSCAD for .scad file conversion

### File Type Associations
Extension handles `.urdf`, `.xacro`, and `.scad` files with:
- JSON schemas for validation (`schemas/urdf-schema.json`, `schemas/xacro-schema.json`)
- Snippets for common patterns (`snippets/xacro-snippets.json`, `snippets/openscad-snippets.json`)
- Custom editors for 3D mesh files (`.stl`, `.dae`)
- tmLanguage grammar for OpenSCAD syntax highlighting (`syntaxes/openscad.tmLanguage.json`)
- OpenSCAD files are automatically converted to STL for 3D preview using `openscad-wasm-prebuilt`

## Common Development Tasks

### Adding New URDF/Xacro Features
1. Update schemas in `schemas/` for validation
2. Modify `utils.ts` if new parsing logic needed
3. Update webview renderer in `src/webview/webview.ts` for visualization
4. Add test cases in `src/test/testdata/`

### Extending Package Resolution
The `rospackCommands` object in `utils.ts` handles ROS package commands. Extend for new substitution patterns:
```typescript
parser.rospackCommands = {
  find: (packageName) => packageMap.get(packageName),
  // Add new commands here
};
```

## MCP Server Integration

The extension includes a Model Context Protocol (MCP) server (`src/mcp.ts`) that exposes tools for AI assistants:

### Available MCP Tools
- **`take_screenshot`**: Captures screenshots of active URDF/Xacro/OpenSCAD previews for visual verification
- **`take_screenshot_by_filename`**: Takes screenshot of a specific file by filename, opening a preview if needed
- **`get_openscad_libraries`**: Provides comprehensive documentation of available OpenSCAD libraries, modules, and functions

### MCP Server Features
- **Auto-start**: Starts automatically when first preview is opened
- **HTTP Transport**: Uses HTTP transport on configurable port (default: 3005)
- **Session Management**: Supports multiple concurrent AI assistant sessions
- **Error Handling**: Robust error handling with detailed logging to output channel

### Configuration Settings
Extension settings are defined in `package.json` under `contributes.configuration`. All settings prefixed with `urdf-editor.` and include visual appearance, camera, debug options, and OpenSCAD library paths. Key settings:
- `urdf-editor.OpenSCADLibraryPaths`: Array of additional library paths for OpenSCAD files. Supports `${workspace}` variable substitution.

## Geometry Guidelines
Per `prompts/urdf-instructions.md`:
- Use basic geometry (spheres, cylinders, cubes) over complex meshes
- Don't reference non-existent 3D files
- Minimize joint count in robot models
- Suggest Xacro macros for duplicate geometry
- Avoid unnecessary transforms