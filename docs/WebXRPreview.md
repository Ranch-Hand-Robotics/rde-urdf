# WebXR URDF Preview
The Robot Developer Extension for URDF allows previewing a URDF file in Virtual Reality. 

## Prerequisites
- A VR headset connected to your computer. The extension has been tested with the Meta Quest 2 and 3.
- [Microsoft DevTunnel Tool](https://learn.microsoft.com/en-us/azure/developer/dev-tunnels/get-started)

## Developer Tunnel
To view the URDF WebXR Preview, you need to set up a developer tunnel. This is a secure connection between your local machine and the VR headset. 

Follow these steps:
1. In any terminal, run the command `devtunnel user login --github` to start the developer tunnel. This will create a secure connection to your local machine.
2. After logging in, run the command `>devtunnel host -p 3000` to create a tunnel.  This will generate a URL that you can use to view the URDF from the VR headset.
3. Type this URL into your headset's browser. This will open the WebXR Preview page.
4. Click on the WebXR Headset button in the lower right to enter the VR environment. This will load the URDF file in the VR headset.



