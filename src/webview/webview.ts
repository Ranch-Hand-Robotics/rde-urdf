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
  private scene: BABYLON.Scene;
  private camera: BABYLON.ArcRotateCamera;
  private gizmoScene: BABYLON.Scene;
  private gizmoCamera: BABYLON.ArcRotateCamera;
  private gizmoCube: BABYLON.Mesh | null = null;
  private canvas: HTMLCanvasElement;
  private engine: BABYLON.Engine;
  
  constructor(mainScene: BABYLON.Scene, mainCamera: BABYLON.ArcRotateCamera) {
    this.scene = mainScene;
    this.camera = mainCamera;
    
    // Create a separate canvas for the gizmo in the bottom-right corner
    this.canvas = document.createElement('canvas');
    this.canvas.id = 'viewGizmoCanvas';
    this.canvas.style.position = 'absolute';
    this.canvas.style.bottom = '20px';
    this.canvas.style.right = '20px';
    this.canvas.style.width = '120px';
    this.canvas.style.height = '120px';
    this.canvas.style.pointerEvents = 'auto';
    this.canvas.style.zIndex = '1000';
    this.canvas.style.borderRadius = '8px';
    this.canvas.style.backgroundColor = 'rgba(0, 0, 0, 0.1)';
    document.body.appendChild(this.canvas);
    
    // Create a separate engine and scene for the gizmo
    this.engine = new BABYLON.Engine(this.canvas, true, { preserveDrawingBuffer: true, stencil: true });
    this.gizmoScene = new BABYLON.Scene(this.engine);
    this.gizmoScene.clearColor = new BABYLON.Color4(0, 0, 0, 0); // Transparent background
    
    // Create camera for gizmo scene
    this.gizmoCamera = new BABYLON.ArcRotateCamera(
      'gizmoCamera',
      0,
      0,
      3,
      BABYLON.Vector3.Zero(),
      this.gizmoScene
    );
    this.gizmoCamera.attachControl(this.canvas, false);
    this.gizmoCamera.inputs.clear(); // Disable camera controls
  }
  
  create(): void {
    // Create a light for the gizmo scene
    const light = new BABYLON.HemisphericLight('gizmoLight', new BABYLON.Vector3(0, 1, 0), this.gizmoScene);
    light.intensity = 1.2;
    
    // Create the cube with labeled faces
    this.gizmoCube = BABYLON.MeshBuilder.CreateBox('gizmoCube', { size: 1 }, this.gizmoScene);
    
    // Create materials for each face with labels and axis directions (Right-handed)
    const faceData = [
      // World axis colors: X = red, Y = green, Z = blue (right-handed)
      { name: 'Front', axis: 'Z+', color: new BABYLON.Color3(0, 0, 1), faceId: 0 },   // +Z = Blue
      { name: 'Back', axis: 'Z-', color: new BABYLON.Color3(0, 0, 1), faceId: 1 },    // -Z = Blue
      { name: 'Right', axis: 'X+', color: new BABYLON.Color3(1, 0, 0), faceId: 2 },   // +X = Red
      { name: 'Left', axis: 'X-', color: new BABYLON.Color3(1, 0, 0), faceId: 3 },    // -X = Red
      { name: 'Top', axis: 'Y+', color: new BABYLON.Color3(0, 1, 0), faceId: 4 },     // +Y = Green
      { name: 'Bottom', axis: 'Y-', color: new BABYLON.Color3(0, 1, 0), faceId: 5 }   // -Y = Green
    ];
    
    // Create multi-material for the cube
    const multiMat = new BABYLON.MultiMaterial('gizmoMultiMat', this.gizmoScene);
    
    faceData.forEach((face, index) => {
      const mat = new BABYLON.StandardMaterial(`gizmoMat${face.name}`, this.gizmoScene);
      mat.diffuseColor = face.color;
      mat.emissiveColor = face.color.scale(0.3);
      mat.specularColor = new BABYLON.Color3(0.2, 0.2, 0.2);
      multiMat.subMaterials.push(mat);
    });
    
    this.gizmoCube.material = multiMat;
    
    // Assign face IDs to submeshes
    this.gizmoCube.subMeshes = [];
    const verticesCount = this.gizmoCube.getTotalVertices();
    
    // Box has 6 faces, 4 vertices per face
    for (let i = 0; i < 6; i++) {
      const subMesh = new BABYLON.SubMesh(i, 0, verticesCount, i * 6, 6, this.gizmoCube);
      this.gizmoCube.subMeshes.push(subMesh);
    }
    
    // Add text labels using GUI
    this.createFaceLabels(faceData);
    
    // Handle click events for face selection
    this.setupClickHandler();
    
    // Start render loop for gizmo
    this.engine.runRenderLoop(() => {
      // Sync gizmo orientation with main camera
      this.syncOrientation();
      this.gizmoScene.render();
    });
    
    // Handle resize
    window.addEventListener('resize', () => {
      this.engine.resize();
    });
  }
  
  private createFaceLabels(faceData: Array<{ name: string; axis: string; color: BABYLON.Color3; faceId: number }>): void {
    // Create text labels directly on the cube faces using DynamicTextures
    const positions: { [key: string]: BABYLON.Vector3 } = {
      'Front': new BABYLON.Vector3(0, 0, 0.5),
      'Back': new BABYLON.Vector3(0, 0, -0.5),
      'Right': new BABYLON.Vector3(0.5, 0, 0),
      'Left': new BABYLON.Vector3(-0.5, 0, 0),
      'Top': new BABYLON.Vector3(0, 0.5, 0),
      'Bottom': new BABYLON.Vector3(0, -0.5, 0)
    };
    
    faceData.forEach(face => {
      // Create a plane for each label positioned on the face
      const plane = BABYLON.MeshBuilder.CreatePlane(`label${face.name}`, { size: 0.6 }, this.gizmoScene);
      plane.position = positions[face.name];
      plane.parent = this.gizmoCube;
      
      // Orient the plane to face outward from cube center
      if (face.name === 'Front') plane.rotation.y = 0;
      if (face.name === 'Back') plane.rotation.y = Math.PI;
      if (face.name === 'Right') plane.rotation.y = Math.PI / 2;
      if (face.name === 'Left') plane.rotation.y = -Math.PI / 2;
      if (face.name === 'Top') plane.rotation.x = -Math.PI / 2;
      if (face.name === 'Bottom') plane.rotation.x = Math.PI / 2;
      
      // Create a dynamic texture for this plane
      const dynamicTexture = new BABYLON.DynamicTexture(`labelTexture${face.name}`, 512, this.gizmoScene);
      const ctx = dynamicTexture.getContext() as CanvasRenderingContext2D;
      
      // Draw background in face color
      ctx.fillStyle = face.color.toHexString();
      ctx.fillRect(0, 0, 512, 512);
      
      // Draw text in white
      ctx.fillStyle = '#FFFFFF';
      ctx.font = 'bold 48px Arial';
      (ctx as any).textAlign = 'center';
      (ctx as any).textBaseline = 'middle';
      ctx.fillText(`${face.name}`, 256, 200);
      ctx.fillText(`(${face.axis})`, 256, 280);
      
      dynamicTexture.update();
      
      // Create material with the texture
      const planeMat = new BABYLON.StandardMaterial(`labelMat${face.name}`, this.gizmoScene);
      planeMat.emissiveTexture = dynamicTexture;
      planeMat.disableLighting = true;
      plane.material = planeMat;
    });
  }
  
  private syncOrientation(): void {
    // Match gizmo cube rotation to main camera's view direction
    // ArcRotateCamera angles: alpha is azimuth (0=+Z, PI/2=-X), beta is elevation (0=top, PI=bottom)
    // Gizmo cube rotation should show same axis orientation as the main scene
    if (this.gizmoCube && this.camera) {
      const alpha = this.camera.alpha;
      const beta = this.camera.beta;
      
      // Map camera angles to rotation:
      // - Y rotation (pitch) matches alpha (camera azimuth)
      // - X rotation (roll) is inverted beta so cube matches camera perspective
      this.gizmoCube.rotation.y = alpha;
      this.gizmoCube.rotation.x = -(beta - Math.PI / 2);
    }
  }
  
  private setupClickHandler(): void {
    // Add pointer down handler directly to canvas for reliable click capture
    this.canvas.addEventListener('pointerdown', (evt) => {
      const rect = this.canvas.getBoundingClientRect();
      const x = evt.clientX - rect.left;
      const y = evt.clientY - rect.top;
      
      // Normalize to canvas coordinates (0-1)
      const normalizedX = x / rect.width;
      const normalizedY = y / rect.height;
      
      // Create ray from camera through the click point
      const camera = this.gizmoCamera;
      if (!camera) return;
      
      // Use createRay or castRay depending on BabylonJS version
      const ray = BABYLON.Ray.Zero();
      ray.direction = BABYLON.Vector3.TransformCoordinates(
        new BABYLON.Vector3(normalizedX * 2 - 1, -(normalizedY * 2 - 1), 1),
        BABYLON.Matrix.Invert(camera.getProjectionMatrix(true))
      );
      ray.origin = camera.position;
      
      // Check if ray hits the gizmo cube
      const hit = this.gizmoScene.pickWithRay(ray, (mesh) => mesh === this.gizmoCube);
      if (hit && hit.hit && hit.subMeshId !== undefined) {
        this.snapToView(hit.subMeshId);
      }
    });
    
    // Add hover effect
    this.canvas.addEventListener('pointermove', (evt) => {
      const rect = this.canvas.getBoundingClientRect();
      const x = evt.clientX - rect.left;
      const y = evt.clientY - rect.top;
      
      const normalizedX = x / rect.width;
      const normalizedY = y / rect.height;
      
      const camera = this.gizmoCamera;
      if (!camera) return;
      
      const ray = BABYLON.Ray.Zero();
      ray.direction = BABYLON.Vector3.TransformCoordinates(
        new BABYLON.Vector3(normalizedX * 2 - 1, -(normalizedY * 2 - 1), 1),
        BABYLON.Matrix.Invert(camera.getProjectionMatrix(true))
      );
      ray.origin = camera.position;
      
      const hit = this.gizmoScene.pickWithRay(ray, (mesh) => mesh === this.gizmoCube);
      this.canvas.style.cursor = (hit && hit.hit) ? 'pointer' : 'default';
    });
  }
  
  private snapToView(faceId: number): void {
    if (!this.camera) return;
    
    // Define standard view positions (alpha, beta, radius)
    const views: { [key: number]: { alpha: number; beta: number } } = {
      0: { alpha: 0, beta: Math.PI / 2 },           // Front (looking at +Z)
      1: { alpha: Math.PI, beta: Math.PI / 2 },     // Back (looking at -Z)
      2: { alpha: Math.PI / 2, beta: Math.PI / 2 }, // Right (looking at -X)
      3: { alpha: -Math.PI / 2, beta: Math.PI / 2 }, // Left (looking at +X)
      4: { alpha: 0, beta: 0 },                     // Top (looking down)
      5: { alpha: 0, beta: Math.PI }                // Bottom (looking up)
    };
    
    const targetView = views[faceId];
    if (!targetView) return;
    
    // Animate camera to target position
    BABYLON.Animation.CreateAndStartAnimation(
      'cameraAlpha',
      this.camera,
      'alpha',
      60,
      30,
      this.camera.alpha,
      targetView.alpha,
      BABYLON.Animation.ANIMATIONLOOPMODE_CONSTANT
    );
    
    BABYLON.Animation.CreateAndStartAnimation(
      'cameraBeta',
      this.camera,
      'beta',
      60,
      30,
      this.camera.beta,
      targetView.beta,
      BABYLON.Animation.ANIMATIONLOOPMODE_CONSTANT
    );
    
    // Reset radius to default
    const defaultRadius = 5;
    BABYLON.Animation.CreateAndStartAnimation(
      'cameraRadius',
      this.camera,
      'radius',
      60,
      30,
      this.camera.radius,
      defaultRadius,
      BABYLON.Animation.ANIMATIONLOOPMODE_CONSTANT
    );
    
    vscode?.postMessage({
      command: "trace",
      text: `Snapped to view: face ${faceId}`,
    });
  }
}

  // Just like a regular webpage we need to wait for the webview
  // DOM to load before we can reference any of the HTML elements
  // or toolkit components
  window.addEventListener("load", main);
  
