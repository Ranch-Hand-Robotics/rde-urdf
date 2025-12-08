# Xacro Configuration Feature

## Overview

The URDF Editor extension supports passing arguments and environment variables to Xacro files during preview without modifying the source files. This is accomplished through a configuration file at `.vscode/xacro.json`.

## Use Cases

- **Testing different robot configurations**: Try different parameter values without editing the source
- **Environment-specific paths**: Use different mesh or resource paths per environment
- **Multi-variant robots**: Configure different robot variants from the same Xacro file
- **CI/CD integration**: Provide configuration values specific to build environments

## Configuration File Format

### Location
`.vscode/xacro.json` in your workspace root

### Structure

```json
{
  "version": "1.0.0",
  "${workspaceFolder}/urdf/robot.xacro": {
    "args": {
      "robot_name": "my_robot",
      "base_width": "0.5",
      "wheel_count": "4"
    },
    "env": {
      "ROBOT_PREFIX": "test",
      "MESH_PATH": "/meshes"
    }
  },
  "**/*.xacro": {
    "args": {
      "default_color": "0.8 0.2 0.2 1.0"
    }
  }
}
```

### Fields

- **version**: Configuration schema version (currently "1.0.0")
- **File patterns**: Keys matching file paths or glob patterns
  - Can be absolute paths
  - Can use `${workspaceFolder}` variable
  - Supports wildcards: `**/*.xacro`, `robot_*.xacro`, etc.
- **args**: Object mapping argument names to values
- **env**: Object mapping environment variable names to values

## How It Works

### Detection Phase

When you preview a Xacro file, the extension:

1. Reads the file content
2. Detects `xacro:arg` declarations
3. Detects `$(arg ...)` usage
4. Detects `$(env ...)` and `$(optenv ...)` usage
5. If any are found and no config exists, prompts user to create one

### Processing Phase

During Xacro processing:

1. Configuration is loaded from `.vscode/xacro.json`
2. File path is matched against patterns in config
3. Arguments are passed to XacroParser via `parser.arguments`
4. Environment variables are resolved:
   - First, check the config file
   - If not found, check `process.env`
   - For `optenv`, use default value if not found

### Pattern Matching

Files are matched using the following precedence:

1. **Exact path match**: Full file path matches exactly
2. **Glob pattern match**: File path matches wildcard pattern
3. **Filename match**: Just the filename matches pattern

The extension uses the `minimatch` library for pattern matching.

## Supported Xacro Patterns

### Arguments

```xml
<!-- Declaration -->
<xacro:arg name="robot_name" default="default_robot"/>

<!-- Usage -->
<link name="$(arg robot_name)_base"/>
<xacro:property name="name_value" value="$(arg robot_name)"/>
```

### Environment Variables

```xml
<!-- Required environment variable -->
<xacro:property name="prefix" value="$(env ROBOT_PREFIX)"/>

<!-- Optional environment variable with default -->
<xacro:property name="color" value="$(optenv ROBOT_COLOR 0.5 0.5 0.5 1.0)"/>
```

## Example Workflow

### 1. Create Xacro File with Arguments

`robot.xacro`:
```xml
<?xml version="1.0"?>
<robot name="configurable_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="robot_name" default="default_robot"/>
  <xacro:arg name="base_size" default="0.5"/>
  
  <xacro:property name="mesh_path" value="$(optenv MESH_PATH /default/meshes)"/>
  
  <link name="$(arg robot_name)_base">
    <visual>
      <geometry>
        <box size="$(arg base_size) $(arg base_size) 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### 2. Preview the File

Right-click the file and select "Preview" or use `Ctrl+Shift+P` → "URDF: Preview"

### 3. Extension Detects Arguments

The extension will show a prompt:
> "The file contains 2 xacro argument(s) and 1 environment variable(s). Would you like to create a configuration file to provide values?"

Select "Yes"

### 4. Configuration File is Created

`.vscode/xacro.json` is created with detected arguments:

```json
{
    "version": "1.0.0",
    "${workspaceFolder}/robot.xacro": {
        "args": {
            "robot_name": "",
            "base_size": ""
        },
        "env": {
            "MESH_PATH": ""
        }
    }
}
```

### 5. Fill in Values

Edit the configuration file:

```json
{
    "version": "1.0.0",
    "${workspaceFolder}/robot.xacro": {
        "args": {
            "robot_name": "my_custom_robot",
            "base_size": "0.8"
        },
        "env": {
            "MESH_PATH": "/opt/robot/meshes"
        }
    }
}
```

### 6. Preview Updates Automatically

Save the configuration file and the preview will update with the new values.

## Advanced Usage

### Multiple File Configurations

```json
{
  "version": "1.0.0",
  "${workspaceFolder}/robots/robot_a.xacro": {
    "args": {
      "robot_name": "robot_a",
      "color": "1.0 0.0 0.0 1.0"
    }
  },
  "${workspaceFolder}/robots/robot_b.xacro": {
    "args": {
      "robot_name": "robot_b",
      "color": "0.0 1.0 0.0 1.0"
    }
  }
}
```

### Wildcard Patterns for Shared Defaults

```json
{
  "version": "1.0.0",
  "**/*.xacro": {
    "env": {
      "MESH_PATH": "/opt/meshes",
      "CONFIG_PATH": "/opt/config"
    }
  },
  "${workspaceFolder}/special_robot.xacro": {
    "args": {
      "special_param": "value"
    },
    "env": {
      "MESH_PATH": "/special/meshes"  // Overrides wildcard
    }
  }
}
```

### Testing Multiple Variants

```json
{
  "version": "1.0.0",
  "${workspaceFolder}/robot.xacro": {
    "args": {
      "variant": "indoor",
      "sensor_count": "2",
      "camera_enabled": "true"
    }
  }
}
```

Change `variant` to "outdoor" and re-preview to see the outdoor configuration.

## Implementation Details

### Module: `xacroConfig.ts`

Key functions:
- `detectArgumentsAndEnv(content)`: Detects args and env vars in xacro content
- `loadXacroConfig()`: Loads configuration from `.vscode/xacro.json`
- `saveXacroConfig(config)`: Saves configuration to file
- `findConfigForFile(filePath, config)`: Finds matching config for file
- `promptCreateConfig()`: Prompts user to create configuration

### Integration: `utils.ts`

In `processXacro()`:
1. Load configuration before parsing
2. Detect arguments and environment variables
3. Prompt user if needed
4. Pass `fileConfig.args` to `parser.arguments`
5. Enhanced `env` and `optenv` callbacks to check config first

### Dependencies

- `minimatch`: Glob pattern matching for file paths
- `xacro-parser`: Supports `parser.arguments` property

## Troubleshooting

### Configuration Not Being Applied

1. Check that `.vscode/xacro.json` exists in workspace root
2. Verify JSON syntax is valid
3. Ensure file path pattern matches your file
4. Check VS Code output channel "URDF Editor" for errors

### Wildcard Pattern Not Matching

1. Use forward slashes `/` in patterns, even on Windows
2. Test pattern with simpler wildcards first: `*.xacro`
3. Try absolute path if relative doesn't work
4. Remember `**` matches any number of directories

### Environment Variable Not Resolved

1. Check spelling in both Xacro file and config
2. Verify config value is not empty string `""`
3. For `optenv`, ensure default value is provided in Xacro
4. Check VS Code output channel for resolution details

## Future Enhancements

Potential future improvements:
- GUI configuration editor
- Validation of argument values
- Auto-completion for argument names
- Configuration templates
- Per-file override settings
