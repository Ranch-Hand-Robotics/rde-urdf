import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from '@polyhobbyist/babylon_ros';
import {Robot} from '@polyhobbyist/babylon_ros';
import * as ColladaFileLoader from '@polyhobbyist/babylon-collada-loader';
import * as GUI from 'babylonjs-gui';

// Get access to the VS Code API from within the webview context
let vscode : any | undefined = undefined;
if (typeof acquireVsCodeApi !== 'undefined') {
  vscode = acquireVsCodeApi();
}

let currentRobotScene : urdf.RobotScene | undefined = undefined;
let advancedTexturePlane: BABYLON.Mesh | undefined = undefined;
let advancedTexture: GUI.AdvancedDynamicTexture | undefined = undefined;
let xrHelper: BABYLON.WebXRDefaultExperience | undefined = undefined;
  

async function apply3DFile(filename: string) {
  if (currentRobotScene === undefined) {
    return;
  }

  currentRobotScene.clearAxisGizmos();
  currentRobotScene.clearRotationGizmos();
  currentRobotScene.clearStatus();
  currentRobotScene.resetCamera();

  if (currentRobotScene.currentRobot) {
    currentRobotScene.currentRobot.dispose();
    currentRobotScene.currentRobot = undefined;
  }

  vscode?.postMessage({
    command: "trace",
    text: `loading 3D file ${filename}`,
  });

  try {
    if (currentRobotScene.scene) {
      let scale = BABYLON.Vector3.One();
      let m = new urdf.Mesh(filename, scale);
      currentRobotScene.currentRobot = new urdf.Robot();
      
      let visual = new urdf.Visual();
      visual.material = new urdf.Material();
      visual.material.name = "default";
      visual.geometry = m;

      let link = new urdf.Link();
      link.visuals.push(visual);

      currentRobotScene.currentRobot.links.set("base_link", link);
      currentRobotScene.currentRobot.create(currentRobotScene.scene);
    }
  } catch (err: any) {
    vscode?.postMessage({
      command: "error",
      text: err.message,
    });
    return;
  }

  vscode?.postMessage({
    command: "trace",
    text: `loaded 3D file ${filename}`, 
  });
}


// Helper function to initialize WebXR with all available features
async function initializeWebXR(scene: BABYLON.Scene) {
  // Create default XR experience with enhanced options
  const xrOptions = {
    uiOptions: {
      sessionMode: "immersive-vr" as XRSessionMode, // Default to AR mode
      onError: (error: any) => {
        if (vscode) {
          vscode.postMessage({
          command: "error",
          text: `WebXR error: ${error.message}`});
        } else {
          console.error("WebXR error:", error.message);
          alert(`WebXR error: ${error.message}`);
        }
      }
    },
    optionalFeatures: true
  };

  // Create the XR experience helper
  xrHelper = await scene.createDefaultXRExperienceAsync(xrOptions);
  
  // Create the texture plane immediately - adjust dimensions to match UI aspect ratio
  // Set height to match 700 pixels with proper aspect ratio
  advancedTexturePlane = BABYLON.MeshBuilder.CreatePlane("UIPlane", { width: 0.09, height: 0.09 }, scene);
  
  // Create the advanced dynamic texture with a fixed size of 700 pixels height
  advancedTexture = GUI.AdvancedDynamicTexture.CreateForMesh(
    advancedTexturePlane,
    200,
    200
  );

  // Add an event listener to reposition UI when entering XR mode
  if (xrHelper?.baseExperience) {
    xrHelper.baseExperience.onStateChangedObservable.add((state) => {
        positionPlaneInFrontOfUser();
    });
  }
}

function positionPlaneInFrontOfUser() {
  if (advancedTexturePlane && currentRobotScene && currentRobotScene.scene && currentRobotScene.scene.activeCamera) {
    // Position the plane
    // Get the active camera - either the XR camera (if in immersive mode) or the regular camera
    let camera : BABYLON.Camera;
    let forward: BABYLON.Vector3;
    if (xrHelper?.baseExperience?.state === BABYLON.WebXRState.IN_XR && xrHelper.baseExperience.camera) {
      camera = xrHelper.baseExperience.camera;
      // Invert the direction for VR camera to fix backwards direction issue
      forward = camera.getForwardRay().direction.scale(-1);
    } else {
      camera = currentRobotScene.scene.activeCamera!;
      forward = camera.getForwardRay().direction;
    }
    advancedTexturePlane.position = camera.position.add(forward.scale(0.5));
    
    // Make plane face the camera
    advancedTexturePlane.lookAt(currentRobotScene.scene.activeCamera.position);
    
    // Offset to the right and top
    const right = BABYLON.Vector3.Cross(forward, BABYLON.Vector3.Up()).normalize();
    const up = BABYLON.Vector3.Cross(right, forward).normalize();

    // Default offsets for desktop view
    let rightOffset = 0.12;
    let upOffset = 0.15;

    // Check if we're in immersive VR mode
    if (xrHelper?.baseExperience?.state === BABYLON.WebXRState.IN_XR) {
      // Use half values for immersive view
      rightOffset = 0.06;
      upOffset = 0.075;
    } else {
      // Adjust rightOffset if debug UI is enabled for desktop view
      if (!currentRobotScene.scene.debugLayer.isVisible()) {
        rightOffset = 0.22;
      }
    }

    advancedTexturePlane.position.addInPlace(right.scale(rightOffset));
    advancedTexturePlane.position.addInPlace(up.scale(upOffset));

  }
};



// Main function that gets executed once the webview DOM loads
async function main() {
  const canvas = document.getElementById("renderCanvas");
  const canvasElement = canvas as unknown as HTMLCanvasElement;

  currentRobotScene = new urdf.RobotScene();
  currentRobotScene.createScene(canvasElement);
  if (currentRobotScene.scene === undefined || currentRobotScene.engine === undefined) {
    return;
  }

  // Initialize WebXR with all features
  await initializeWebXR(currentRobotScene.scene);

  currentRobotScene.createUI(advancedTexture);

  positionPlaneInFrontOfUser();
  currentRobotScene.scene.onBeforeRenderObservable.add(positionPlaneInFrontOfUser);
  

  currentRobotScene.engine.runRenderLoop(function () {
    if (currentRobotScene !== undefined && currentRobotScene.scene !== undefined) {
      currentRobotScene.scene.render();
    }
  });
  
  currentRobotScene.engine.resize();
  
  window.addEventListener("resize", function () {
    if (currentRobotScene !== undefined && currentRobotScene.engine !== undefined) {
      currentRobotScene.engine.resize();
    }
  });  

  window.addEventListener('message', event => {
    if (currentRobotScene === undefined) {
      return;
    }
    
    const message = event.data; // The JSON data our extension sent
    switch (message.command) {
        case 'view3DFile':
          apply3DFile(message.filename);
        break;
        case 'urdf':
          currentRobotScene.applyURDF(message.urdf);
        break;
        case 'previewFile':
        vscode.setState({previewFile: message.previewFile});
        break;
        case 'colors':
          if (currentRobotScene.camera && currentRobotScene.ground && currentRobotScene.scene) {
            currentRobotScene.camera.radius = message.cameraRadius;
            currentRobotScene.scene.clearColor = BABYLON.Color4.FromHexString(message.backgroundColor);
            let gm = currentRobotScene.ground.material as Materials.GridMaterial;
            
            // These are flipped on purpose
            gm.lineColor = BABYLON.Color3.FromHexString(message.gridMainColor);
            gm.mainColor = BABYLON.Color3.FromHexString(message.gridLineColor);
            gm.minorUnitVisibility = parseFloat(message.gridMinorOpacity);
            gm.gridRatio = parseFloat(message.gridRatio);
            gm.majorUnitFrequency = parseFloat(message.majorUnitFrequency);

            if (message.debugUI) {
              currentRobotScene.scene.debugLayer.show();
            } else {
              currentRobotScene.scene.debugLayer.hide();
            }

            currentRobotScene.readyToRender = true;
          }
          break;
    }
  });

  vscode?.postMessage({
    command: "ready"
  });

}

// Just like a regular webpage we need to wait for the webview
// DOM to load before we can reference any of the HTML elements
// or toolkit components
window.addEventListener("load", main);
