# Change Log

All notable changes to the "urdf-editor" extension will be documented in this file.

Check [Keep a Changelog](http://keepachangelog.com/) for recommendations on how to structure this file.

## [Unreleased]

### Added
- VSCode settings for GitHub Copilot code generation
- Comprehensive test data for URDF/Xacro parsing and validation
- Test files covering simple to complex robot configurations

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