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

type CustomizerValue = string | number | boolean | number[];

interface CustomizerOption {
  label?: string;
  value: string | number;
}

interface CustomizerVariable {
  name: string;
  valueType: 'string' | 'number' | 'boolean' | 'vector';
  defaultValue: CustomizerValue;
  tab: string;
  description?: string;
  widget: 'dropdown' | 'slider' | 'checkbox' | 'spinbox' | 'textbox' | 'vector';
  options?: CustomizerOption[];
  range?: {
    min?: number;
    max?: number;
    step?: number;
  };
  maxLength?: number;
}

interface CustomizerParseWarning {
  line: number;
  message: string;
}

interface CustomizerModelPayload {
  variables: CustomizerVariable[];
  warnings: CustomizerParseWarning[];
}

let currentCustomizerModel: CustomizerModelPayload | undefined;
let currentCustomizerValues: Record<string, CustomizerValue> = {};
let customizerAutoPreview = true;
let customizerDebounceHandle: number | undefined;

function forceCanvasRelayout() {
  if (!currentRobotScene?.engine) {
    return;
  }

  // Wait for DOM/CSS layout to settle, then resize Babylon engine.
  requestAnimationFrame(() => {
    currentRobotScene?.engine?.resize();
    // One extra frame improves reliability with flexbox transitions.
    requestAnimationFrame(() => currentRobotScene?.engine?.resize());
  });
}

function debounceCustomizerApply() {
  if (!customizerAutoPreview) {
    return;
  }

  if (customizerDebounceHandle) {
    window.clearTimeout(customizerDebounceHandle);
  }

  customizerDebounceHandle = window.setTimeout(() => {
    vscode?.postMessage({
      command: 'openscadCustomizerSetValues',
      values: currentCustomizerValues,
      autoPreview: customizerAutoPreview,
      applyNow: true
    });
  }, 350);
}

function updateCustomizerWarnings(warnings: CustomizerParseWarning[]) {
  const warningsElement = document.getElementById('customizerWarnings');
  if (!warningsElement) {
    return;
  }

  if (warnings.length === 0) {
    warningsElement.textContent = '';
    return;
  }

  warningsElement.innerHTML = '';
  const title = document.createElement('div');
  title.textContent = `Warnings (${warnings.length})`;
  title.style.fontWeight = 'bold';
  title.style.marginBottom = '4px';
  warningsElement.appendChild(title);

  for (const warning of warnings) {
    const row = document.createElement('div');
    row.textContent = warning.line > 0
      ? `Line ${warning.line}: ${warning.message}`
      : warning.message;
    warningsElement.appendChild(row);
  }
}

function createScalarInput(variable: CustomizerVariable, value: CustomizerValue): HTMLElement {
  let input: HTMLInputElement | HTMLSelectElement;

  if (variable.widget === 'dropdown' && variable.options && variable.options.length > 0) {
    const select = document.createElement('select');
    for (const option of variable.options) {
      const opt = document.createElement('option');
      opt.value = `${option.value}`;
      opt.textContent = option.label || `${option.value}`;
      select.appendChild(opt);
    }
    select.value = `${value}`;
    select.addEventListener('change', () => {
      currentCustomizerValues[variable.name] = variable.valueType === 'number'
        ? Number(select.value)
        : select.value;
      debounceCustomizerApply();
    });
    input = select;
  } else {
    const textInput = document.createElement('input');
    input = textInput;

    switch (variable.widget) {
      case 'slider':
        textInput.type = 'range';
        if (variable.range?.min !== undefined) textInput.min = `${variable.range.min}`;
        if (variable.range?.max !== undefined) textInput.max = `${variable.range.max}`;
        if (variable.range?.step !== undefined) textInput.step = `${variable.range.step}`;
        textInput.value = `${value}`;

        const sliderWrapper = document.createElement('div');
        sliderWrapper.style.display = 'grid';
        sliderWrapper.style.gridTemplateColumns = '1fr auto';
        sliderWrapper.style.alignItems = 'center';
        sliderWrapper.style.gap = '8px';

        const sliderValue = document.createElement('span');
        sliderValue.textContent = textInput.value;
        sliderValue.style.minWidth = '3ch';
        sliderValue.style.textAlign = 'right';
        sliderValue.style.fontSize = '12px';
        sliderValue.style.opacity = '0.9';

        const updateSliderValue = () => {
          sliderValue.textContent = textInput.value;
          currentCustomizerValues[variable.name] = Number(textInput.value);
          debounceCustomizerApply();
        };

        textInput.addEventListener('input', updateSliderValue);
        textInput.addEventListener('change', updateSliderValue);

        sliderWrapper.appendChild(textInput);
        sliderWrapper.appendChild(sliderValue);
        return sliderWrapper;
      case 'checkbox':
        textInput.type = 'checkbox';
        textInput.checked = Boolean(value);
        textInput.style.width = 'auto';
        textInput.addEventListener('change', () => {
          currentCustomizerValues[variable.name] = textInput.checked;
          debounceCustomizerApply();
        });
        break;
      case 'spinbox':
        textInput.type = 'number';
        if (variable.range?.min !== undefined) textInput.min = `${variable.range.min}`;
        if (variable.range?.max !== undefined) textInput.max = `${variable.range.max}`;
        if (variable.range?.step !== undefined) textInput.step = `${variable.range.step}`;
        textInput.value = `${value}`;
        textInput.addEventListener('change', () => {
          currentCustomizerValues[variable.name] = Number(textInput.value);
          debounceCustomizerApply();
        });
        break;
      default:
        textInput.type = 'text';
        if (variable.maxLength !== undefined) {
          textInput.maxLength = variable.maxLength;
        }
        textInput.value = `${value}`;
        textInput.addEventListener('change', () => {
          currentCustomizerValues[variable.name] = textInput.value;
          debounceCustomizerApply();
        });
        break;
    }
  }

  input.setAttribute('data-customizer-name', variable.name);
  return input;
}

function createVectorInput(variable: CustomizerVariable, value: CustomizerValue): HTMLElement {
  const wrapper = document.createElement('div');
  wrapper.style.display = 'grid';
  wrapper.style.gridTemplateColumns = 'repeat(4, minmax(40px, 1fr))';
  wrapper.style.gap = '4px';

  const vector = Array.isArray(value) ? value.slice(0, 4) : (Array.isArray(variable.defaultValue) ? variable.defaultValue.slice(0, 4) : [0]);

  for (let i = 0; i < vector.length; i++) {
    const input = document.createElement('input');
    input.type = 'number';
    if (variable.range?.min !== undefined) input.min = `${variable.range.min}`;
    if (variable.range?.max !== undefined) input.max = `${variable.range.max}`;
    if (variable.range?.step !== undefined) input.step = `${variable.range.step}`;
    input.value = `${vector[i]}`;
    input.setAttribute('data-customizer-name', variable.name);
    input.setAttribute('data-customizer-vector-index', `${i}`);
    input.addEventListener('change', () => {
      const current = Array.isArray(currentCustomizerValues[variable.name])
        ? [...(currentCustomizerValues[variable.name] as number[])]
        : [...vector];
      current[i] = Number(input.value);
      currentCustomizerValues[variable.name] = current;
      debounceCustomizerApply();
    });
    wrapper.appendChild(input);
  }

  return wrapper;
}

function renderCustomizer(model: CustomizerModelPayload, overrides: Record<string, CustomizerValue>, enabled: boolean, autoPreview: boolean) {
  const pane = document.getElementById('customizerPane');
  const variablesContainer = document.getElementById('customizerVariables');
  const autoPreviewInput = document.getElementById('customizerAutoPreview') as HTMLInputElement | null;

  if (!pane || !variablesContainer || !autoPreviewInput) {
    return;
  }

  const hasCustomizableContent = enabled && Array.isArray(model.variables) && model.variables.length > 0;
  const wasVisible = pane.classList.contains('visible');

  pane.classList.toggle('visible', hasCustomizableContent);
  if (wasVisible !== hasCustomizableContent) {
    forceCanvasRelayout();
  }

  if (!hasCustomizableContent) {
    variablesContainer.innerHTML = '';
    updateCustomizerWarnings([]);
    currentCustomizerModel = undefined;
    currentCustomizerValues = {};
    return;
  }

  currentCustomizerModel = model;
  currentCustomizerValues = {};
  customizerAutoPreview = autoPreview;
  autoPreviewInput.checked = autoPreview;

  variablesContainer.innerHTML = '';

  const grouped = new Map<string, CustomizerVariable[]>();
  for (const variable of model.variables) {
    const tab = variable.tab || 'parameters';
    if (!grouped.has(tab)) {
      grouped.set(tab, []);
    }
    grouped.get(tab)?.push(variable);
  }

  for (const [tabName, variables] of grouped) {
    const tabTitle = document.createElement('div');
    tabTitle.className = 'customizer-tab-title';
    tabTitle.textContent = tabName;
    variablesContainer.appendChild(tabTitle);

    for (const variable of variables) {
      const field = document.createElement('div');
      field.className = 'customizer-field';

      const label = document.createElement('label');
      label.textContent = variable.description
        ? `${variable.name} — ${variable.description}`
        : variable.name;
      field.appendChild(label);

      const value = overrides[variable.name] ?? variable.defaultValue;
      currentCustomizerValues[variable.name] = value;

      if (variable.widget === 'vector' || variable.valueType === 'vector') {
        field.appendChild(createVectorInput(variable, value));
      } else {
        field.appendChild(createScalarInput(variable, value));
      }

      variablesContainer.appendChild(field);
    }
  }

  updateCustomizerWarnings(model.warnings || []);
}

function resetCustomizerValues() {
  if (!currentCustomizerModel) {
    return;
  }

  for (const variable of currentCustomizerModel.variables) {
    currentCustomizerValues[variable.name] = variable.defaultValue;
  }

  renderCustomizer(currentCustomizerModel, currentCustomizerValues, true, customizerAutoPreview);
}

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

  const customizerApplyButton = document.getElementById('customizerApply');
  const customizerResetButton = document.getElementById('customizerReset');
  const customizerAutoPreviewInput = document.getElementById('customizerAutoPreview') as HTMLInputElement | null;

  customizerApplyButton?.addEventListener('click', () => {
    vscode?.postMessage({
      command: 'openscadCustomizerSetValues',
      values: currentCustomizerValues,
      autoPreview: customizerAutoPreview,
      applyNow: true
    });
  });

  customizerResetButton?.addEventListener('click', () => {
    resetCustomizerValues();
    vscode?.postMessage({
      command: 'openscadCustomizerReset'
    });
  });

  customizerAutoPreviewInput?.addEventListener('change', () => {
    customizerAutoPreview = customizerAutoPreviewInput.checked;
    vscode?.postMessage({
      command: 'openscadCustomizerSetAutoPreview',
      value: customizerAutoPreview
    });

    if (customizerAutoPreview) {
      debounceCustomizerApply();
    }
  });
  
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
        vscode.setState({
          previewFile: message.previewFile,
          customizerValues: currentCustomizerValues,
          customizerAutoPreview: customizerAutoPreview
        });
        break;

        case 'openscadCustomizerModel':
          renderCustomizer(
            message.model || { variables: [], warnings: [] },
            message.overrides || {},
            Boolean(message.enabled),
            message.autoPreview ?? true
          );
          break;

        case 'openscadCustomizerResetValues':
          resetCustomizerValues();
        break;
        case 'colors':
          if (currentRobotScene.camera && currentRobotScene.ground && currentRobotScene.scene) {
            // Use new API methods if available, otherwise fall back to direct property access
            const robotSceneAny = currentRobotScene as any;
            
            // Set default camera position using new API
            robotSceneAny.setDefaultCameraPosition({
              alpha: message.defaultCameraAlpha,
              beta: message.defaultCameraBeta,
              radius: message.defaultCameraRadius
            });
            
            // Also set the current camera position to match the defaults
            currentRobotScene.camera.alpha = message.defaultCameraAlpha;
            currentRobotScene.camera.beta = message.defaultCameraBeta;
            
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
}

  // Just like a regular webpage we need to wait for the webview
  // DOM to load before we can reference any of the HTML elements
  // or toolkit components
  window.addEventListener("load", main);
  
