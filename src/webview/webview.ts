import * as BABYLON from 'babylonjs';
import * as Materials from 'babylonjs-materials';
import * as urdf from '@ranchhandrobotics/babylon_ros';
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
      
      // Apply scaling for files that use mm as default units
      // OpenSCAD exports STL in mm, so scale down to meters for robotics
      // GLB, GLTF, DAE typically use meters already
      if (filename.toLowerCase().endsWith('.stl')) {
        // STL files from OpenSCAD are in mm, scale down to meters
        scale = new BABYLON.Vector3(0.001, 0.001, 0.001);
      }
      
      let m = new urdf.Mesh(filename, scale);
      currentRobotScene.currentRobot = new urdf.Robot();
      
      let visual = new urdf.Visual();
      visual.material = new urdf.Material();
      visual.material.name = "default";
      visual.material.color = new BABYLON.Color4(.8, .8, .8, 1); // Bright white for better visibility
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
            // Use new API methods if available, otherwise fall back to direct property access
            const robotSceneAny = currentRobotScene as any;
            
            if (typeof robotSceneAny.setCameraRadius === 'function') {
              robotSceneAny.setCameraRadius(message.cameraRadius);
            } else {
              currentRobotScene.camera.radius = message.cameraRadius;
            }

            if (typeof robotSceneAny.setBackgroundColor === 'function') {
              robotSceneAny.setBackgroundColor(message.backgroundColor);
            } else {
              currentRobotScene.scene.clearColor = BABYLON.Color4.FromHexString(message.backgroundColor);
            }
            
            if (typeof robotSceneAny.setGridProperties === 'function') {
              // Grid colors are flipped on purpose in the original implementation
              robotSceneAny.setGridProperties({
                lineColor: message.gridMainColor,
                mainColor: message.gridLineColor,
                minorOpacity: parseFloat(message.gridMinorOpacity),
                gridRatio: parseFloat(message.gridRatio),
                majorUnitFrequency: parseFloat(message.majorUnitFrequency)
              });
            } else {
              // Fallback to direct material property access
              let gm = currentRobotScene.ground.material as Materials.GridMaterial;
              // These are flipped on purpose
              gm.lineColor = BABYLON.Color3.FromHexString(message.gridMainColor);
              gm.mainColor = BABYLON.Color3.FromHexString(message.gridLineColor);
              gm.minorUnitVisibility = parseFloat(message.gridMinorOpacity);
              gm.gridRatio = parseFloat(message.gridRatio);
              gm.majorUnitFrequency = parseFloat(message.majorUnitFrequency);
            }

            // Configure mirror properties based on reflectivity setting
            const mirrorReflectivity = parseFloat(message.mirrorReflectivity) || 0;
            
            if (typeof robotSceneAny.setMirrorProperties === 'function') {
              if (mirrorReflectivity > 0) {
                // Create toned-down color from grid main color
                const gridMainColor = BABYLON.Color3.FromHexString(message.gridMainColor);
                const tonedDownColor = gridMainColor.scale(0.3); // Reduce intensity by 70%
                const tintColorHex = tonedDownColor.toHexString();
                
                vscode?.postMessage({
                  command: "trace",
                  text: `Setting mirror properties: reflectivity=${mirrorReflectivity}, tintColor=${tintColorHex}`,
                });
                
                robotSceneAny.setMirrorProperties({
                  reflectionLevel: 0.3,
                  alpha: 0.4,
                  tintColor: tintColorHex,
                  blurKernel: 16,
                  roughness: 0.6,
                  enabled: true
                });
              } else {
                // Disable mirror when reflectivity is 0
                vscode?.postMessage({
                  command: "trace",
                  text: "Disabling mirror (reflectivity is 0)",
                });
                
                robotSceneAny.setMirrorProperties({
                  enabled: false
                });
              }
            } else {
              // API not available yet
              if (mirrorReflectivity > 0) {
                vscode?.postMessage({
                  command: "trace",
                  text: `Mirror reflectivity setting (${mirrorReflectivity}) received but setMirrorProperties API not available in RobotScene yet`,
                });
              }
            }

            if (message.debugUI) {
              currentRobotScene.scene.debugLayer.show();
            } else {
              currentRobotScene.scene.debugLayer.hide();
            }

            currentRobotScene.readyToRender = true;
          }
          break;

          case 'takeScreenshot':
            if (currentRobotScene.camera && currentRobotScene.scene) {
              const width = message.width || 1024;
              const height = message.height || 1024;

              currentRobotScene.takeScreenshot(width, height).then((base64Image) => {
                vscode?.postMessage({
                  command: "screenshotResult",
                  base64Image: base64Image,
                  width: width,
                  height: height,
                  success: true,
                });
              }).catch((error) => {
                vscode?.postMessage({
                  command: "screenshotResult",
                  width: width,
                  height: height,
                  success: false,
                  text: `Failed to take screenshot: ${error instanceof Error ? error.message : String(error)}`,
                });
              });
            }
            break;
    }
  });

  vscode?.postMessage({
    command: "ready"
  });

  const xrHelper = await currentRobotScene.scene.createDefaultXRExperienceAsync();
  
  // Create and setup the view gizmo
  if (currentRobotScene.scene && currentRobotScene.camera) {
    const viewGizmo = new ViewGizmo(currentRobotScene.scene, currentRobotScene.camera);
    viewGizmo.create();
  }
}

/**
 * ViewGizmo - A 3D orientation cube that shows current camera orientation
 * and allows clicking faces to snap camera to standard views
 */
class ViewGizmo {
  private mainScene: BABYLON.Scene;
  private mainCamera: BABYLON.ArcRotateCamera;
  private gizmoScene: BABYLON.Scene;
  private gizmoCamera: BABYLON.ArcRotateCamera;
  private gizmoCube: BABYLON.Mesh | null = null;
  private gizmoCanvas: HTMLCanvasElement;
  private gizmoEngine: BABYLON.Engine;
  private faceData: Map<BABYLON.AbstractMesh, number> = new Map();
  
  constructor(mainScene: BABYLON.Scene, mainCamera: BABYLON.ArcRotateCamera) {
    this.mainScene = mainScene;
    this.mainCamera = mainCamera;
    
    // Create overlay canvas for gizmo - smaller and positioned in corner
    const container = document.createElement('div');
    container.id = 'gizmoContainer';
    container.style.position = 'fixed';
    container.style.bottom = '20px';
    container.style.right = '20px';
    container.style.width = '140px';
    container.style.height = '140px';
    container.style.zIndex = '10000';
    container.style.pointerEvents = 'auto';
    document.body.appendChild(container);
    
    this.gizmoCanvas = document.createElement('canvas');
    this.gizmoCanvas.id = 'gizmoCanvas';
    this.gizmoCanvas.width = 140;
    this.gizmoCanvas.height = 140;
    this.gizmoCanvas.style.borderRadius = '8px';
    this.gizmoCanvas.style.border = '2px solid #666';
    this.gizmoCanvas.style.cursor = 'grab';
    this.gizmoCanvas.style.display = 'block';
    this.gizmoCanvas.style.pointerEvents = 'auto';
    container.appendChild(this.gizmoCanvas);
    
    // Create separate engine/scene for gizmo
    this.gizmoEngine = new BABYLON.Engine(this.gizmoCanvas, true);
    this.gizmoScene = new BABYLON.Scene(this.gizmoEngine);
    this.gizmoScene.clearColor = new BABYLON.Color4(0.15, 0.15, 0.15, 1);
    
    // Create camera for gizmo scene
    this.gizmoCamera = new BABYLON.ArcRotateCamera(
      'gizmoCamera',
      0, 
      Math.PI / 2.5,
      2.5,
      BABYLON.Vector3.Zero(),
      this.gizmoScene
    );
    this.gizmoCamera.attachControl(this.gizmoCanvas, true);
    this.gizmoCamera.inputs.clear();
  }
  
  create(): void {
    // Add light to gizmo scene
    const light = new BABYLON.HemisphericLight('light', new BABYLON.Vector3(1, 1, 1), this.gizmoScene);
    light.intensity = 1.5;
    
    // Create parent mesh for the cube
    this.gizmoCube = new BABYLON.Mesh('gizmoCube', this.gizmoScene);
    
    // Define faces with colors matching standard axis colors
    const faces = [
      { name: 'Front', pos: [0, 0, 0.5], axis: 'Z+', color: new BABYLON.Color3(0, 0, 1), faceId: 0 },
      { name: 'Back', pos: [0, 0, -0.5], axis: 'Z-', color: new BABYLON.Color3(0, 0, 0.6), faceId: 1 },
      { name: 'Right', pos: [0.5, 0, 0], axis: 'X+', color: new BABYLON.Color3(1, 0, 0), faceId: 2 },
      { name: 'Left', pos: [-0.5, 0, 0], axis: 'X-', color: new BABYLON.Color3(0.6, 0, 0), faceId: 3 },
      { name: 'Top', pos: [0, 0.5, 0], axis: 'Y+', color: new BABYLON.Color3(0, 1, 0), faceId: 4 },
      { name: 'Bottom', pos: [0, -0.5, 0], axis: 'Y-', color: new BABYLON.Color3(0, 0.6, 0), faceId: 5 }
    ];
    
    // Create each face as a plane
    faces.forEach(face => {
      // Create plane for face
      const plane = BABYLON.MeshBuilder.CreatePlane(`${face.name}_plane`, { size: 0.9 }, this.gizmoScene);
      plane.parent = this.gizmoCube;
      plane.position = new BABYLON.Vector3(face.pos[0], face.pos[1], face.pos[2]);
      
      // Rotate plane to face outward
      if (face.name === 'Back') plane.rotation.y = Math.PI;
      else if (face.name === 'Right') plane.rotation.y = -Math.PI / 2;
      else if (face.name === 'Left') plane.rotation.y = Math.PI / 2;
      else if (face.name === 'Top') plane.rotation.x = Math.PI / 2;
      else if (face.name === 'Bottom') plane.rotation.x = -Math.PI / 2;
      
      // Create material with texture
      const mat = new BABYLON.StandardMaterial(`mat_${face.name}`, this.gizmoScene);
      mat.diffuseColor = face.color;
      mat.specularColor = new BABYLON.Color3(0.2, 0.2, 0.2);
      
      // Create dynamic texture with text
      const texture = new BABYLON.DynamicTexture(`tex_${face.name}`, 256, this.gizmoScene);
      const ctx = texture.getContext() as CanvasRenderingContext2D;
      
      // Draw colored background
      ctx.fillStyle = face.color.toHexString();
      ctx.fillRect(0, 0, 256, 256);
      
      // Draw text
      ctx.fillStyle = '#FFFFFF';
      ctx.shadowColor = '#000000';
      ctx.shadowBlur = 4;
      ctx.shadowOffsetX = 2;
      ctx.shadowOffsetY = 2;
      
      ctx.font = 'bold 32px Arial';
      (ctx as any).textAlign = 'center';
      (ctx as any).textBaseline = 'middle';
      ctx.fillText(face.name, 128, 90);
      
      ctx.font = 'bold 24px Arial';
      ctx.fillText(`(${face.axis})`, 128, 150);
      
      texture.update();
      
      // Assign texture and material
      mat.emissiveTexture = texture;
      plane.material = mat;
      
      // Store face ID for picking
      this.faceData.set(plane, face.faceId);
    });
    
    // Setup click handler
    this.setupClickHandler();
    
    // Render loop
    this.gizmoEngine.runRenderLoop(() => {
      this.syncOrientation();
      this.gizmoScene.render();
    });
    
    window.addEventListener('resize', () => {
      this.gizmoEngine.resize();
    });
  }
  
  private syncOrientation(): void {
    // Sync gizmo camera with main camera
    if (this.gizmoCamera && this.mainCamera) {
      this.gizmoCamera.alpha = this.mainCamera.alpha;
      this.gizmoCamera.beta = this.mainCamera.beta;
    }
  }
  
  private setupClickHandler(): void {
    this.gizmoCanvas.addEventListener('click', (event: MouseEvent) => {
      // Use scene's built-in picking
      const pickResult = this.gizmoScene.pick(event.offsetX, event.offsetY, (mesh) => {
        return this.gizmoCube?.getChildren().includes(mesh) || false;
      });
      
      if (pickResult && pickResult.hit && pickResult.pickedMesh) {
        const faceId = this.faceData.get(pickResult.pickedMesh);
        if (faceId !== undefined) {
          this.snapToView(faceId);
        }
      }
    });
    
    // Cursor feedback
    this.gizmoCanvas.addEventListener('mousemove', (event: MouseEvent) => {
      const pickResult = this.gizmoScene.pick(event.offsetX, event.offsetY, (mesh) => {
        return this.gizmoCube?.getChildren().includes(mesh) || false;
      });
      
      this.gizmoCanvas.style.cursor = (pickResult && pickResult.hit) ? 'pointer' : 'grab';
    });
  }
  
  private snapToView(faceId: number): void {
    const views: { [key: number]: { alpha: number; beta: number } } = {
      0: { alpha: 0, beta: Math.PI / 2 },
      1: { alpha: Math.PI, beta: Math.PI / 2 },
      2: { alpha: Math.PI / 2, beta: Math.PI / 2 },
      3: { alpha: -Math.PI / 2, beta: Math.PI / 2 },
      4: { alpha: 0, beta: 0 },
      5: { alpha: 0, beta: Math.PI }
    };
    
    const target = views[faceId];
    if (!target) return;
    
    // Animate main camera
    BABYLON.Animation.CreateAndStartAnimation(
      'snapAlpha',
      this.mainCamera,
      'alpha',
      60,
      30,
      this.mainCamera.alpha,
      target.alpha,
      BABYLON.Animation.ANIMATIONLOOPMODE_CONSTANT
    );
    
    BABYLON.Animation.CreateAndStartAnimation(
      'snapBeta',
      this.mainCamera,
      'beta',
      60,
      30,
      this.mainCamera.beta,
      target.beta,
      BABYLON.Animation.ANIMATIONLOOPMODE_CONSTANT
    );
  }
}

// Just like a regular webpage we need to wait for the webview
// DOM to load before we can reference any of the HTML elements
// or toolkit components
window.addEventListener("load", main);
