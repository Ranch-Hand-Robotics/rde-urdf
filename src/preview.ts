// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Additional features added by PolyHobbyist

import * as vscode from 'vscode';
import * as path from 'path';
import * as util from './utils';
import { trace } from 'console';
import * as fs from 'fs';
import * as os from 'os';
import {createOpenSCAD} from 'openscad-wasm-prebuilt';

// Type declaration for openscad-wasm
interface OpenSCADInstance {
  FS: {
    writeFile(path: string, data: string | Uint8Array): void;
    readFile(path: string, options: { encoding: string }): string;
  };
  callMain(args: string[]): void;
}

interface OpenSCAD {
  getInstance(): OpenSCADInstance;
}

interface OpenSCADModule {
  createOpenSCAD(options?: {
    print?: (text: string) => void;
    printErr?: (text: string) => void;
  }): Promise<OpenSCAD>;
}

// Helper functions for OpenSCAD library support
function getDefaultOpenSCADLibraryPaths(): string[] {
  const platform = os.platform();
  const homeDir = os.homedir();
  const paths: string[] = [];

  switch (platform) {
    case 'win32':
      // Windows: My Documents\OpenSCAD\libraries
      const documentsPath = path.join(homeDir, 'Documents', 'OpenSCAD', 'libraries');
      paths.push(documentsPath);
      break;
    case 'linux':
      // Linux: $HOME/.local/share/OpenSCAD/libraries
      const linuxLibPath = path.join(homeDir, '.local', 'share', 'OpenSCAD', 'libraries');
      paths.push(linuxLibPath);
      break;
    case 'darwin':
      // macOS: $HOME/Documents/OpenSCAD/libraries
      const macLibPath = path.join(homeDir, 'Documents', 'OpenSCAD', 'libraries');
      paths.push(macLibPath);
      break;
  }

  return paths;
}

function resolveWorkspaceVariables(libraryPath: string, workspaceRoot?: string): string {
  if (workspaceRoot && libraryPath.includes('${workspace}')) {
    return libraryPath.replace(/\$\{workspace\}/g, workspaceRoot);
  }
  return libraryPath;
}

async function getAllOpenSCADLibraryPaths(workspaceRoot?: string): Promise<string[]> {
  const config = vscode.workspace.getConfiguration('urdf-editor');
  const userLibraryPaths: string[] = config.get('OpenSCADLibraryPaths', []);
  
  // Get default OS-specific paths
  const defaultPaths = getDefaultOpenSCADLibraryPaths();
  
  // Resolve workspace variables in user paths
  const resolvedUserPaths = userLibraryPaths.map(p => resolveWorkspaceVariables(p, workspaceRoot));
  
  // Combine all paths
  const allPaths = [...defaultPaths, ...resolvedUserPaths];
  
  // Filter to only existing directories
  const existingPaths: string[] = [];
  for (const libPath of allPaths) {
    try {
      const stat = await fs.promises.stat(libPath);
      if (stat.isDirectory()) {
        existingPaths.push(libPath);
      }
    } catch {
      // Directory doesn't exist, skip it
    }
  }
  
  return existingPaths;
}

async function loadLibraryFiles(instance: OpenSCADInstance, libraryPaths: string[], trace: vscode.OutputChannel): Promise<void> {
  for (const libPath of libraryPaths) {
    try {
      trace.appendLine(`Loading OpenSCAD library from: ${libPath}`);
      await loadLibraryDirectory(instance, libPath, '', trace);
    } catch (error) {
      trace.appendLine(`Failed to load library from ${libPath}: ${error}`);
    }
  }
}

async function loadLibraryDirectory(instance: OpenSCADInstance, basePath: string, relativePath: string, trace: vscode.OutputChannel): Promise<void> {
  const fullPath = path.join(basePath, relativePath);
  
  try {
    const entries = await fs.promises.readdir(fullPath, { withFileTypes: true });
    
    for (const entry of entries) {
      const entryPath = path.join(relativePath, entry.name);
      const fullEntryPath = path.join(fullPath, entry.name);
      
      if (entry.isDirectory()) {
        // Recursively load subdirectories
        await loadLibraryDirectory(instance, basePath, entryPath, trace);
      } else if (entry.isFile() && (entry.name.endsWith('.scad') || entry.name.endsWith('.stl') || entry.name.endsWith('.dxf'))) {
        // Load library files into the virtual filesystem
        try {
          if (entry.name.endsWith('.scad')) {
            const content = await fs.promises.readFile(fullEntryPath, 'utf8');
            const virtualPath = `/libraries/${entryPath}`;
            instance.FS.writeFile(virtualPath, content);
            trace.appendLine(`Loaded library file: ${virtualPath}`);
          } else {
            // For binary files like STL, DXF
            const content = await fs.promises.readFile(fullEntryPath);
            const virtualPath = `/libraries/${entryPath}`;
            instance.FS.writeFile(virtualPath, new Uint8Array(content));
            trace.appendLine(`Loaded binary library file: ${virtualPath}`);
          }
        } catch (error) {
          trace.appendLine(`Failed to load library file ${entryPath}: ${error}`);
        }
      }
    }
  } catch (error) {
    trace.appendLine(`Failed to read directory ${fullPath}: ${error}`);
  }
}

export default class URDFPreview 
{
    private _resource: vscode.Uri;
    private _processing: boolean;
    private  _context: vscode.ExtensionContext;
    private _disposables: vscode.Disposable[] = [];
    private _urdfEditor: vscode.TextEditor | null = null;
    _webview: vscode.WebviewPanel | undefined = undefined;
    private _trace: vscode.OutputChannel;
    private _convertedSTLPath: string | undefined = undefined;

    private _pendingScreenshots: Array<{
        width: number;
        height: number;
        resolve: (value: string) => void;
        reject: (reason?: any) => void;
    }> = [];

    public get state() {
        return {
            resource: this.resource.toString()
        };
    }

    public static create(
        context: vscode.ExtensionContext,
        resource: vscode.Uri,
        trace: vscode.OutputChannel
        ) : URDFPreview
    {

        var paths : vscode.Uri[] = [];

        // Create paths from workspace folders to vscode uri, and add extensions
        var workspaceFolders = vscode.workspace.workspaceFolders;
        if (workspaceFolders) {
            workspaceFolders.forEach((folder) => {
                paths.push(vscode.Uri.file(folder.uri.fsPath));
            });
        }

        paths.push(vscode.Uri.file(path.join(context.extensionPath, 'dist')));
        paths.push(vscode.Uri.file(path.join(context.extensionPath, 'node_modules/@polyhobbyist/babylon_ros/dist')));
        paths.push(vscode.Uri.file(path.join(context.extensionPath, 'node_modules/babylonjs')));

        
        // Create and show a new webview
        var editor = vscode.window.createWebviewPanel(
            'urdfPreview_standalone', // Identifies the type of the webview. Used internally
            'URDF Preview', // Title of the panel displayed to the user
            vscode.ViewColumn.Two, // Editor column to show the new webview panel in.
            { 
                enableScripts: true,
                retainContextWhenHidden: true,
                localResourceRoots: paths
            }
        );

        return new URDFPreview(editor, context, resource, trace);
    }

    public takeScreenshot(width: number, height: number): Promise<string> {
        return new Promise((resolve, reject) => {
            if (!this._webview) {
                reject('Webview is not initialized');
                return;
            }

            this._pendingScreenshots.push({
                width,
                height,
                resolve,
                reject
            });

            this._webview.webview.postMessage({
                command: 'takeScreenshot',
                width,
                height
            });
        });
    }

    private constructor(
        webview: vscode.WebviewPanel,
        context: vscode.ExtensionContext,
        resource: vscode.Uri,
        trace: vscode.OutputChannel
    )
    {
        this._webview = webview;
        this._context = context;
        this._resource = resource;
        this._trace = trace;
        this._processing = false;

        let subscriptions: vscode.Disposable[] = [];

            // Set an event listener to listen for messages passed from the webview context
        this._setWebviewMessageListener(this._webview.webview);

        this._webview.webview.html = this._getWebviewContent(this._webview.webview, context.extensionUri);

        vscode.workspace.onDidSaveTextDocument(event => {

            if (event && this.isPreviewOf(event.uri)) {
                this.refresh();
            }
        }, this, this._context.subscriptions);

        this._webview.onDidDispose(() => {
            this.dispose();
        }, null, this._context.subscriptions);        

        vscode.workspace.onDidChangeConfiguration(event => {
            this.updateColors();
        }, this, this._context.subscriptions);


        this._disposables = subscriptions;
    }

    // Convert OpenSCAD file to STL
    private async convertOpenSCADToSTL(scadFilePath: string): Promise<string | null> {
        try {
            this._trace.appendLine(`Starting OpenSCAD conversion for: ${scadFilePath}`);
            
            const openscad = await createOpenSCAD({
                // Optional callbacks for stdout/stderr
                print: (text: string) => this._trace.appendLine(`OpenSCAD: ${text}`),
                printErr: (text: string) => this._trace.appendLine(`OpenSCAD Error: ${text}`),
            });

            // Get direct access to the WASM module
            const instance = openscad.getInstance();

            // Get workspace root for variable resolution
            const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
            
            // Load OpenSCAD libraries
            this._trace.appendLine(`Loading OpenSCAD libraries...`);
            const libraryPaths = await getAllOpenSCADLibraryPaths(workspaceRoot);
            if (libraryPaths.length > 0) {
                this._trace.appendLine(`Found ${libraryPaths.length} library paths: ${libraryPaths.join(', ')}`);
                await loadLibraryFiles(instance, libraryPaths, this._trace);
            } else {
                this._trace.appendLine(`No OpenSCAD library paths found`);
            }

            const basename = path.basename(scadFilePath, '.scad');
            const dir = path.dirname(scadFilePath);
            const stlPath = path.join(dir, `${basename}.stl`);
            
            this._trace.appendLine(`Converting OpenSCAD to STL: ${stlPath}`);

            const scadText = await fs.promises.readFile(scadFilePath, 'utf8');
            
            // Write the main SCAD file to the virtual filesystem
            instance.FS.writeFile('/input.scad', scadText);
            
            // Add library search paths to OpenSCAD arguments
            const args = ['-o', '/output.stl', '/input.scad'];
            if (libraryPaths.length > 0) {
                // Add each library path as a -L option
                for (const libPath of libraryPaths) {
                    args.unshift('-L', '/libraries');
                }
            }
            
            // Run OpenSCAD with library paths
            instance.callMain(args);
            
            // Read the output STL from the virtual filesystem
            const stlContent = instance.FS.readFile('/output.stl', { encoding: 'binary' });

            // Use the filesystem API directly
            await fs.promises.writeFile(stlPath, stlContent, 'binary');

            this._convertedSTLPath = stlPath;
            
            this._trace.appendLine(`OpenSCAD conversion completed successfully: ${stlPath}`);
            return stlPath;
        } catch (error) {
            this._trace.appendLine(`OpenSCAD conversion failed: ${error}`);
            vscode.window.showErrorMessage(`Failed to convert OpenSCAD file: ${error}`);
            return null;
        }
    }

    // Check if file is OpenSCAD
    private isOpenSCADFile(filePath: string): boolean {
        return path.extname(filePath).toLowerCase() === '.scad';
    }

    public get resource(): vscode.Uri {
        return this._resource;
    }

    public async refresh() {
        if (!this._processing) {
            this.loadResource();
        }
    }

    private updateColors() {
        if (this._webview) {
            const config = vscode.workspace.getConfiguration("urdf-editor");
            this._webview.webview.postMessage({ 
                command: 'colors', 
                cameraRadius: config.get("CameraDistanceToRobot", "1.0"),
                backgroundColor: config.get("BackgroundColor", "#000000"),
                gridLineColor: config.get("GridMinorColor", "#00FF00"),
                gridMainColor: config.get("GridMainColor", "#001100"),
                gridMinorOpacity: config.get("GridMinorOpacity", "0.4"),
                majorUnitFrequency: config.get("GridFrequency", "5"),
                gridRatio: config.get("GridRatio", "0.1"),
                debugUI: config.get("DebugUI", "false"),
            });
        }
    }

    private async loadResource()  {
        this._processing = true;

        if (!this._webview) {
            this._processing = false;
            this._trace.appendLine("Webview is not available, cannot load resource.");
            return;
        }

        try {
            // Check if this is an OpenSCAD file
            if (this.isOpenSCADFile(this._resource.fsPath)) {
                // Handle OpenSCAD file - convert to STL and display as 3D model
                const stlPath = await this.convertOpenSCADToSTL(this._resource.fsPath);
                if (stlPath) {
                    const stlUri = vscode.Uri.file(stlPath);
                    
                    this._trace.appendLine("OpenSCAD file previewing: " + this._resource.toString());
                    
                    this.updateColors();
                    
                    this._webview.webview.postMessage({ command: 'previewFile', previewFile: this._resource.path});
                    this._webview.webview.postMessage({
                        command: 'view3DFile',
                        filename: this._webview.webview.asWebviewUri(stlUri).toString()
                    });
                }
            } else {
                // Handle URDF/Xacro files
                var [urdfText, packagesNotFound] = await util.processXacro(this._resource.fsPath.toString(), 
                    (packageName :vscode.Uri) => {
                        if (!this._webview) {
                            return packageName.fsPath.toString();
                        }
                        // Convert the package name to a webview URI
                        return this._webview.webview.asWebviewUri(packageName).toString();
                    });

                var previewFile = this._resource.toString();

                this._trace.appendLine("URDF previewing: " + previewFile);
                this._trace.append(urdfText);

                this.updateColors();        

                this._webview.webview.postMessage({ command: 'previewFile', previewFile: this._resource.path});
                this._webview.webview.postMessage({ command: 'urdf', urdf: urdfText });

                if (packagesNotFound.length > 0) {
       
                    // Log each package not found
                    for (const pkg of packagesNotFound) {
                        this._trace.appendLine(`\nPackage not found: ${pkg}`);
                    }
                    
                    this._trace.appendLine("\nNOTE: This version of the URDF Renderer will not look for packages outside the workspace.");
                    // Show missing packages as a temporary status bar message
                    vscode.window.setStatusBarMessage(
                        `$(warning) ${packagesNotFound.length} package(s) not found. Check 'URDF Editor' output for details.`, 
                        10000  // show for 10 seconds
                    );
                }
            }

        } catch (err : any) {
            vscode.window.showErrorMessage(err.message);
        } finally {
            this._processing = false;
        }
    }

    public static async revive(
        webview: vscode.WebviewPanel,
        context: vscode.ExtensionContext,
        trace: vscode.OutputChannel,
        state: any,
    ): Promise<URDFPreview> {
        const resource = vscode.Uri.file(state.previewFile);

        const preview = new URDFPreview(
            webview,
            context,
            resource,
            trace);

        return preview;
    }    

    public matchesResource(
        otherResource: vscode.Uri
    ): boolean {
        return this.isPreviewOf(otherResource);
    }

    public reveal() {
        this._webview?.reveal(vscode.ViewColumn.Two);
    }    

    private isPreviewOf(resource: vscode.Uri): boolean {
        return this._resource.fsPath === resource.fsPath;
    }

    private readonly _onDisposeEmitter = new vscode.EventEmitter<void>();
    public readonly onDispose = this._onDisposeEmitter.event;    
    
    private readonly _onDidChangeViewStateEmitter = new vscode.EventEmitter<vscode.WebviewPanelOnDidChangeViewStateEvent>();
    public readonly onDidChangeViewState = this._onDidChangeViewStateEmitter.event;

    public update(resource: vscode.Uri) {
        const editor = vscode.window.activeTextEditor;

        // If we have changed resources, cancel any pending updates
        const isResourceChange = resource.fsPath !== this._resource.fsPath;
        this._resource = resource;
        // Schedule update if none is pending
        this.refresh();
    }
    
    public dispose() {
        while (this._disposables.length) {
            const disposable = this._disposables.pop();
            if (disposable) {
                disposable.dispose();
            }
        }

        // Clean up temporary STL file if it exists
        if (this._convertedSTLPath && fs.existsSync(this._convertedSTLPath)) {
            try {
                fs.unlinkSync(this._convertedSTLPath);
            } catch (error) {
                this._trace.appendLine(`Failed to clean up temporary STL file: ${error}`);
            }
        }

        this._onDisposeEmitter.fire();
        this._onDisposeEmitter.dispose();

        this._onDidChangeViewStateEmitter.dispose();
        this._webview?.dispose();    
        this._webview = undefined;
        this._processing = false;
    }


    /**
   * Defines and returns the HTML that should be rendered within the webview panel.
   *
   * @remarks This is also the place where *references* to CSS and JavaScript files
   * are created and inserted into the webview HTML.
   *
   * @param webview A reference to the extension webview
   * @param extensionUri The URI of the directory containing the extension
   * @returns A template string literal containing the HTML that should be
   * rendered within the webview panel
   */
    private _getWebviewContent(webview: vscode.Webview, extensionUri: vscode.Uri) {
        const webviewUri = util.getUri(webview, extensionUri, ["dist", "webview.js"]);
        const webviewUriSourceMap = util.getUri(webview, extensionUri, ["dist", "webview.js.map"]);
        const webviewUriUrdf = util.getUri(webview, extensionUri, ["node_modules/@polyhobbyist/babylon_ros/dist", "ros.js"]);
        const webviewUriBabylon = util.getUri(webview, extensionUri, ["node_modules/babylonjs", "babylon.max.js"]);
        const nonce = util.getNonce();

        // Tip: Install the es6-string-html VS Code extension to enable code highlighting below
        return /*html*/ `
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <!--meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${webview.cspSource}; font-src ${webview.cspSource}; img-src ${webview.cspSource} https:; script-src 'nonce-${nonce}';"-->
                <style nonce="${nonce}">
                html,
                body {
                overflow: hidden;
                width: 100%;
                height: 100%;
                margin: 0;
                padding: 0;
                }

                #renderCanvas {
                width: 100%;
                height: 100%;
                touch-action: none;
                }
            </style>
            <title>URDF Preview</title>
            </head>
            <body>
                <canvas id="renderCanvas" touch-action="none"></canvas>    
                <script type="module" nonce="${nonce}" src="${webviewUriBabylon}"></script>
                <script type="module" nonce="${nonce}" src="${webviewUri}"></script>
            </body>
            </html>
        `;
    }

    private _setWebviewMessageListener(webview: vscode.Webview) {
        webview.onDidReceiveMessage(
        (message: any) => {
            const command = message.command;
            const text = message.text;
    
            switch (command) {
            case "info":
                vscode.window.showInformationMessage(text);
                return;
            case "error":
                vscode.window.showErrorMessage(text);
                return;
            case "trace":
                this._trace.appendLine(text);
                return;
            case "ready":
                this.refresh();
                return;

            case "screenshotResult":
                // Find and resolve matching screenshot requests
                this._pendingScreenshots = this._pendingScreenshots.filter(pending => {
                    if (pending.width === message.width && pending.height === message.height) {
                        if (message.success) {
                            pending.resolve(message.base64Image);
                        } else {
                            pending.reject(message.text || 'Screenshot failed');
                        }
                        return false; // Remove from array
                    }
                    return true; // Keep in array
                });
            }
        },
        undefined,
        this._disposables
        );
    }
    }
