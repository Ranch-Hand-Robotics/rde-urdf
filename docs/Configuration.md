# Configuration Guide

This guide covers all configuration options available in the URDF Editor extension.

## ROS Package Discovery

The extension automatically discovers ROS packages from multiple sources. Understanding how this works helps you configure the extension for your specific ROS setup.

### How Package Discovery Works

The extension searches for ROS packages by looking for `package.xml` files in the configured directories. For each `package.xml` file found, it:

1. Parses the XML content
2. Extracts the package name from the `<name>` element
3. Maps the package name to the directory containing the `package.xml` file

**Example package.xml structure:**
```xml
<?xml version="1.0"?>
<package format="2">
  <name>robot_state_publisher</name>
  <version>3.0.0</version>
  <description>The robot_state_publisher package</description>
  <!-- ... other elements ... -->
</package>
```

In this example, the package name "robot_state_publisher" would be associated with the directory containing this `package.xml` file.

### Package Search Order

Packages are discovered in this order of precedence:

1. **Workspace Folders** (highest precedence)
   - Current VS Code workspace directories
   - Allows local development overrides

2. **ROS Distro Directory**
   - System-installed ROS packages
   - Configured via `ROS2.distro` and `ROS2.pixiRoot` settings

3. **User-Specified Search Paths**
   - Additional directories you configure
   - Useful for overlay workspaces or custom installations

### ROS Distro Configuration

Configure your ROS distribution using the ROS2 extension settings:

```json
{
  "ROS2.distro": "kilted",
  "ROS2.pixiRoot": "/path/to/pixi/ros/installation"
}
```

- **`ROS2.distro`**: Your ROS distribution name (e.g., "kilted", "humble", "jazzy")
- **`ROS2.pixiRoot`**: Base path for pixi-managed ROS installations

**Path Resolution Logic:**
- If `pixiRoot` is set: uses `${pixiRoot}/share`
- If `pixiRoot` is not set: uses `/opt/ros/${distro}/share`

### Custom Package Search Paths

Add additional directories to search for ROS packages:

```json
{
  "urdf-editor.PackageSearchPaths": [
    "${workspace}/../other_ws/src",
    "/opt/ros/custom_distro/share",
    "/home/user/ros_packages"
  ]
}
```

**Variable Substitution:**
- `${workspace}` → Your workspace root directory

**Use Cases:**
- Overlay workspaces: `"${workspace}/../overlay_ws/src"`
- Custom ROS installations: `"/opt/ros/my_custom_build/share"`
- Shared package repositories: `"/shared/ros_packages"`

## OpenSCAD Configuration

### Library Paths

The extension automatically loads OpenSCAD libraries from OS-specific locations:

- **Windows**: `%USERPROFILE%\Documents\OpenSCAD\libraries`
- **Linux**: `$HOME/.local/share/OpenSCAD/libraries`
- **macOS**: `$HOME/Documents/OpenSCAD/libraries`

Add custom library paths:

```json
{
  "urdf-editor.OpenSCADLibraryPaths": [
    "${workspace}/scad_libs",
    "C:\\MyLibraries\\OpenSCAD",
    "/usr/local/share/openscad/libraries"
  ]
}
```

### Documentation Generation

Generate comprehensive documentation for your OpenSCAD libraries:

1. Open Command Palette (`Ctrl+Shift+P`)
2. Run "URDF: Generate OpenSCAD Libraries Documentation"
3. Choose save location for the markdown file

The generated documentation includes:
- Header comments from library files
- Module and function signatures
- Parameter documentation
- Usage examples

## Visual Configuration

### Background and Grid

```json
{
  "urdf-editor.BackgroundColor": "#000000",
  "urdf-editor.GridMainColor": "#00FF00",
  "urdf-editor.GridMinorColor": "#001100",
  "urdf-editor.GridMinorOpacity": 0.5,
  "urdf-editor.GridRatio": 0.1,
  "urdf-editor.GridFrequency": 10
}
```

### Camera and Mirror

```json
{
  "urdf-editor.CameraDistanceToRobot": 1.0,
  "urdf-editor.MirrorReflectivity": 0.0
}
```

## Debug Configuration

```json
{
  "urdf-editor.DebugUI": false,
  "urdf-editor.mcpServerPort": 3005
}
```

## Example Complete Configuration

```json
{
  // ROS Configuration
  "ROS2.distro": "kilted",
  "ROS2.pixiRoot": "/home/user/pixi/envs/ros_env",

  // Package Search Paths
  "urdf-editor.PackageSearchPaths": [
    "${workspace}/../overlay_ws/src",
    "/opt/ros/custom_packages"
  ],

  // OpenSCAD Libraries
  "urdf-editor.OpenSCADLibraryPaths": [
    "${workspace}/scad_libs",
    "/usr/local/share/openscad/libraries"
  ],

  // Visual Settings
  "urdf-editor.BackgroundColor": "#1a1a1a",
  "urdf-editor.GridMainColor": "#00FF00",
  "urdf-editor.CameraDistanceToRobot": 2.0,

  // Debug
  "urdf-editor.DebugUI": false
}
```