# Change Log

All notable changes to the "urdf-editor" extension will be documented in this file.

Check [Keep a Changelog](http://keepachangelog.com/) for recommendations on how to structure this file.

## [Unreleased]

### Added
- **IntelliSense Features**:
  - Go to Definition (F12) support for OpenSCAD modules and functions across files
  - Go to Definition (F12) support for URDF/Xacro links, joints, macros, properties, and file paths
  - Enhanced hover documentation for OpenSCAD with signatures and parameter descriptions
  - Enhanced hover documentation for URDF/Xacro with inline XML attributes and types
  - Hover documentation for xacro property references (${property_name}) showing values and definitions
  - Property hover searches included files via `<xacro:include>` statements
  - Support for `$(find package_name)` syntax in xacro includes
  - Added hover documentation for `inertia` and `mass` elements
  - Performance optimization: skips getText() on large documents (>10,000 lines)
  - Smart comment extraction for user-defined OpenSCAD modules (filters decorative headers)
  - Cross-file symbol resolution via `include` and `use` statements
  - Package path resolution (package://) for URDF/Xacro files
- **Visual Configuration API**:
  - New `setCameraRadius()` method for programmatic camera control
  - New `setBackgroundColor()` method for scene background customization
  - New `setGridProperties()` method for grid appearance configuration
  - New `setVisualConfig()` method for bulk visual property updates
  - New `setMirrorProperties()` method for ground mirror configuration
  - Mirror reflectivity setting via `urdf-editor.MirrorReflectivity` (0 = disabled, >0 = enabled with opacity)
  - Automatic mirror color derivation from grid main color
- **Documentation Improvements**:
  - AI-assisted development workflow documentation in OpenSCAD.md
  - Expanded MCP integration details with tool descriptions
  - Performance optimization documentation (preview mode, timeout settings)
  - Visual programming paradigm explanation

### Fixed
- Definition provider regex patterns using correct word boundaries instead of line-start anchors
- URDF/Xacro definition provider now works with file patterns regardless of language ID assignment
- OpenSCAD hover provider made async to support cross-file documentation lookup
- Comment extraction now skips section headers and decorative comment lines

### Changed
- Updated webview to use new visual configuration APIs with backward compatibility
- Improved OpenSCAD documentation extraction to ignore ALL CAPS headers and separator lines
- Definition providers now properly escape special regex characters in symbol names
- Enhanced URDF/Xacro hover with full XML signatures showing all attributes inline

## [1.3.1] - 2025-11-21

### Added
- 'scad' keyword tag for better discoverability

### Changed
- Updated version to 1.3.1

## [1.3.0] - 2025-09-18

### Added
- Additional search paths for ROS packages via `urdf-editor.PackageSearchPaths` setting
- Test cases for peer package references
- Xacro snippet for initializing robot templates
- Support for ROS distro and pixi root references in package resolution

### Fixed
- Missing notification messages in UI
- Package resolution with relative paths

### Changed
- Refreshed UI elements and rendering pipeline
- Updated to new BabylonJS rendering system

## [1.2.0] - 2025-08-28

### Added
- **OpenSCAD Support**: Complete OpenSCAD (.scad) file editing, preview, and conversion to URDF/STL
- OpenSCAD syntax highlighting and language configuration
- OpenSCAD snippets for common patterns
- OpenSCAD library support with configurable paths via `urdf-editor.OpenSCADLibraryPaths`
- OpenSCAD worker integration for background processing
- OpenSCAD documentation generation via `urdf-editor.generateOpenSCADDocs` command
- **Model Context Protocol (MCP) Server**: AI assistant integration with tools for screenshots and library documentation
- Screenshot capture tool for URDF/Xacro/OpenSCAD previews
- MCP server commands: `urdf-editor.startMcpServer` and `urdf-editor.stopMcpServer`
- MCP server port configuration via `urdf-editor.mcpServerPort` setting
- Example URDF models for testing
- Compliance guide for GPL-2.0 dependency (OpenSCAD)
- Support for additional 3D file formats: OBJ, GLB, GLTF
- Mirror reflectivity setting via `urdf-editor.MirrorReflectivity`
- Editor title menu integration for quick preview access

### Fixed
- AI-assisted coding error handling improvements
- OpenSCAD conversion cancellation support
- BabylonJS dependency version pinning to avoid breaking changes

### Changed
- Enhanced OpenSCAD documentation with AI-assisted development features
- Improved performance optimizations for 3D rendering
- Updated URDF instruction prompts for better AI assistance
- Refactored code structure for improved readability and maintainability
- Updated babylon_ros to @ranchhandrobotics/babylon_ros v0.1.10
- Added keywords for better marketplace discoverability: ROS, ROS 2, OpenSCAD, Xacro, MCP, WebXR

### Documentation
- Added comprehensive OpenSCAD documentation
- Added MCP integration documentation
- Updated WebXR preview documentation
- Added licensing and compliance information

## [1.1.1] - 2025-08-06

### Added
- OpenVSX marketplace release pipeline
- Preview links in documentation

### Changed
- Moved URDF_Preview documentation
- Updated package dependencies for OpenVSX compatibility

## [1.1.0] - 2025-07-05

### Fixed
- BabylonJS dependencies updated to fixed versions
- Removed unused rospack arg function

### Changed
- Updated dependencies and versions in package.json
- Documentation improvements

## [1.0.1] - 2025-06-24

### Changed
- Removed Mac and Windows specific builds from release pipeline

## [1.0.0] - 2025-05-29

### Added
- Enhanced package dependencies and error handling
- Tests for package URI conversion
- Export tracing output channel for better debugging
- Enhanced error logging in URDF processing

### Fixed
- Resource loading and error handling in URDFPreview
- Missing packages logging
- Path normalization for Windows compatibility
- Issue #38: Improved resource handling

### Changed
- Updated babylon_ros and package dependencies to latest versions

## [0.0.3] - 2025-03-23

### Added
- Documentation support pages and MkDocs configuration
- Support.md documentation file

### Changed
- Updated icon image
- Updated README.md with better documentation
- Improved package.json metadata

## [0.0.2] - 2025-03-20

### Added
- Publishing to VS Code Marketplace
- Schemas for URDF and Xacro files
- Custom instructions and prompts for GitHub Copilot
- Joint visualization improvements

### Fixed
- Syntax highlighting issues
- GitHub Copilot chat configuration

## [0.0.1] - 2025-03-19

### Added
- WebXR preview support for immersive 3D viewing
- WebXR documentation

### Fixed
- Startup issues with initial loading

## [0.0.0] - 2024-12-15

### Added
- URDF Exporter for import into Isaac Sim
- WebXR initial working implementation
- Collision visualization toggle
- Mesh preview support for STL and DAE files
- 3D custom editor for mesh files
- Workflow automation and CI/CD
- Icon for the extension
- Documentation pages with MkDocs

### Fixed
- Material rendering issues
- Standalone mode error isolation
- Parse error handling

### Changed
- Reduced code duplication in preview rendering
- Improved error handling for standalone usage

## Initial Development - 2024-09-27 to 2024-11-03

### Added
- Initial URDF Editor implementation
- Standalone package management
- End-to-end URDF preview functionality
- Context menu commands for URDF/Xacro files
- Background and grid customization settings
- Camera distance configuration
- Debug UI mode

### Infrastructure
- GitHub Actions workflows
- Release automation
- License and README documentation