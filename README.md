# Visual Studio Code ROS URDF Editor
This extension provides developer tooling for Unified Robot Description Files (URDF) and Xacro Editing and Preview. 

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
- AI Assisted Coding prompts fpr Github Copilot
- Virtual Reality preview of the model you are editing.

## Coming Soon
- [Physics Emulation](https://github.com/ranchhandrobotics/vscode_urdf/issues/4)
- [Kinematics Visualization](https://github.com/ranchhandrobotics/vscode_urdf/issues/5)
- [External URDF / Xacro References](https://github.com/ranchhandrobotics/vscode_urdf/issues/6)

## Usage
1. Open a URDF file
2. Right click on the file and select "Preview URDF", or press `Ctrl+Shift+P` and select "Preview URDF"


## Support
If you encounter any issues with this extension, the following resources are provided:
### Github Issues
Bugs and feature requests are handled through [Github Issues in the Repository](https://github.com/Ranch-Hand-Robotics/rde-urdf/issues). 
If you find that you are hitting the same issue as someone else, please give a :+1: or comment on an existing issue.
Please provide as much details as possible, including an isolated reproduction of the issue or a pointer to an online repository.

### Discussions
[Github Discussions](https://github.com/orgs/Ranch-Hand-Robotics/discussions) are provided for community driven general guidance, walkthroughs, or support.

### Sponsored Support
 ![Coming Soon](https://img.shields.io/badge/Coming%20Soon-8A2BE2)
 One on one support, mentoring and consulting will be available through Github Sponsors and Patreon. 

## Acknowledgements
I was the maintainer of Microsoft's [Visual Studio Code ROS Extension](http://aka.ms/ros/vscode). It is no longer maintained, and I've been given permission to externalize the components. The URDF editor was built on work done in my private personal time, and not associated with Microsoft. This extension is a stand alone implementation and will be maintained moving forward.

This extension relis on the [Xacro-Parser](https://www.npmjs.com/package/xacro-parser) by [GKJohnson](https://github.com/gkjohnson) for stand alone Xacro parsing without requiring ROS.

The code for this extension unapologetically uses AI generated code.

## License
Some of the code in this extension is based on the [Visual Studio Code ROS Extension](http://aka.ms/ros/vscode) which is licensed under the MIT License. It also depends on Babylon ROS and Babylon Collada Loader by Polyhobbyist, both of which are MIT licensed.
This extension is also licensed under the MIT License.

