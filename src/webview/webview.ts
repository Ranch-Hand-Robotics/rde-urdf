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

// Main function that gets executed once the webview DOM loads
async function main() {
  const canvas = document.getElementById("renderCanvas");
  const canvasElement = canvas as unknown as HTMLCanvasElement;

  currentRobotScene = new urdf.RobotScene();
  currentRobotScene.createScene(canvasElement);
  if (currentRobotScene.scene === undefined || currentRobotScene.engine === undefined) {
    return;
  }

  currentRobotScene.createUI();
  
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

  const xrHelper = await currentRobotScene.scene.createDefaultXRExperienceAsync();  
}

  // Just like a regular webpage we need to wait for the webview
  // DOM to load before we can reference any of the HTML elements
  // or toolkit components
  window.addEventListener("load", main);
  
