# WebXR URDF 

The Robot Developer Extension for URDF allows previewing a URDF file in Virtual Reality. This feature has been tested with a Meta Quest 3 headset and provides a way to visualize robot models in a 3D environment. 

WebXR requires a secure connection to function properly, which is why we use the Microsoft DevTunnel tool to create a secure tunnel to your local machine, which has the URDF file open. To start the WebXR Preview for a URDF file, you need to follow the steps outlined below.

Microsoft DevTunnel is a tool that allows you to create secure tunnels to your local machine, enabling you to access local resources from remote devices, such as VR headsets. This is particularly useful for WebXR applications, which require HTTPS to function properly.

## Prerequisites

- A VR headset. The extension has been tested with the Meta Quest 3.
- [Microsoft DevTunnel Tool](https://learn.microsoft.com/en-us/azure/developer/dev-tunnels/get-started)

## Developer Tunnel

Follow these steps:

1. In any terminal, run the command `devtunnel user login --github` to start the developer tunnel. Log in with your GitHub account when prompted. This will authenticate you and allow you to create secure tunnels to your local machine.
2. After logging in, run the command `devtunnel host -p 3000` to create a tunnel.  This will generate a URL that you can use to view the URDF from the VR headset in the next step.
3. Type the Generated URL from the previous step into your headset's Web Browser. This will open the WebXR Preview page as a flat HTML page in your VR headset.
4. Click on the WebXR Headset button in the lower right to enter the VR environment. This will load the URDF file in the VR headset.

# Future Work
- Support for more VR headsets.
- Support for more URDF features in the VR environment.
- Improved user interface for the WebXR Preview.
- Integration with other tools and services for a more seamless experience.
- Enhanced performance and stability for larger URDF files.
