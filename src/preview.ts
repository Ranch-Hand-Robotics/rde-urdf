// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Additional features added by PolyHobbyist

import * as vscode from 'vscode';
import * as path from 'path';
import * as util from './utils';
import { trace } from 'console';
import * as fs from 'fs';
import * as os from 'os';
import { convertOpenSCADToSTL, convertOpenSCADToSTLCancellable, isOpenSCADFile } from './openscad';

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
    private _cancellationTokenSource: vscode.CancellationTokenSource | undefined = undefined;

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

        this.updateColors();

        this._disposables = subscriptions;
    }

    // Convert OpenSCAD file to STL with cancellation support
    private async convertOpenSCADToSTL(scadFilePath: string): Promise<string | null> {
        // Cancel any existing conversion
        if (this._cancellationTokenSource) {
            this._trace.appendLine("Cancelling Previous Conversion for " + scadFilePath);
            this._cancellationTokenSource.cancel();
        }
        
        // Create new cancellation token
        this._cancellationTokenSource = new vscode.CancellationTokenSource();
        
        try {
            const result = await convertOpenSCADToSTLCancellable(
                scadFilePath, 
                this._trace, 
                this._cancellationTokenSource.token,
                {
                    previewMode: true,        // Always use preview mode for speed
                    timeout: 60000           // 1 minute timeout for fast feedback
                }
            );
            
            if (result) {
                this._convertedSTLPath = result;
            }
            return result;
        } finally {
            // Clean up cancellation token
            if (this._cancellationTokenSource) {
                this._cancellationTokenSource.dispose();
                this._cancellationTokenSource = undefined;
            }
        }
    }

    // Check if file is OpenSCAD
    private isOpenSCADFile(filePath: string): boolean {
        return isOpenSCADFile(filePath);
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
                mirrorReflectivity: config.get("MirrorReflectivity", 0),
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
        // Cancel any ongoing OpenSCAD conversion
        if (this._cancellationTokenSource) {
            this._cancellationTokenSource.cancel();
            this._cancellationTokenSource.dispose();
            this._cancellationTokenSource = undefined;
        }

        while (this._disposables.length) {
            const disposable = this._disposables.pop();
            if (disposable) {
                disposable.dispose();
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
