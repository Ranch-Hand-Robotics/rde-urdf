# Visual Studio Code ROS URDF Editor
this extension is a stand alone URDF Preview Window which does not require a ROS 2 installation. 

## Features
- 3D preview
- Link Visualization
- Joint Visualization
- Color Customization
- Camera Controls
- Gizmos for exploring a URDF


## Coming Soon
- [Syntax highlighting](https://github.com/ranchhandrobotics/vscode_urdf/issues/1)
- [Code completion](https://github.com/ranchhandrobotics/vscode_urdf/issues/2)
- [Collision Visualization](https://github.com/ranchhandrobotics/vscode_urdf/issues/3)
- [Physics Emulation](https://github.com/ranchhandrobotics/vscode_urdf/issues/4)
- [Kinematics Visualization](https://github.com/ranchhandrobotics/vscode_urdf/issues/5)
- [External URDF / Xacro References](https://github.com/ranchhandrobotics/vscode_urdf/issues/6)

## Usage
1. Open a URDF file
2. Right click on the file and select "Preview URDF", or press `Ctrl+Shift+P` and select "Preview URDF"


# Acknowledgements
I was the maintainer of Microsoft's [Visual Studio Code ROS Extension](http://aka.ms/ros/vscode). It is no longer maintained, and I've been given permission to externalize the components. The URDF editor was built on work done in my private personal time, and not associated with Microsoft. This extension is a stand alone implementation and will be maintained moving forward.

This extension relis on the [Xacro-Parser](https://www.npmjs.com/package/xacro-parser) by [GKJohnson](https://github.com/gkjohnson) for stand alone Xacro parsing without requiring ROS.

# License
Some of the code in this extension is based on the [Visual Studio Code ROS Extension](http://aka.ms/ros/vscode) which is licensed under the MIT License. It also depends on Babylon ROS and Babylon Collada Loader by Polyhobbyist, both of which are MIT licensed.
This extension is also licensed under the MIT License.

