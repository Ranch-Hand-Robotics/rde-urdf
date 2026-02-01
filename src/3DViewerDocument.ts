import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import * as util from './utils';

export class Viewer3DDocument implements vscode.CustomDocument {
	public uri: vscode.Uri;
	public parserPaths: string[] = [];
	public webviewPanel: vscode.WebviewPanel | undefined = undefined;
  private _disposables: vscode.Disposable[] = [];

  protected _context: vscode.ExtensionContext;
  private _trace: vscode.OutputChannel;

  constructor(context: vscode.ExtensionContext, uri: vscode.Uri, trace: vscode.OutputChannel) {
    this._context = context;
    this.uri = uri;
    this._trace = trace;
}

  create(webviewPanel: vscode.WebviewPanel) {
    var paths : vscode.Uri[] = [];

    this.webviewPanel = webviewPanel;

    // Create paths from workspace folders to vscode uri, and add extensions
    var workspaceFolders = vscode.workspace.workspaceFolders;
    if (workspaceFolders) {
        workspaceFolders.forEach((folder) => {
            paths.push(vscode.Uri.file(folder.uri.fsPath));
        });
    }

    paths.push(vscode.Uri.file(path.join(this._context.extensionPath, 'dist')));
    paths.push(vscode.Uri.file(path.join(this._context.extensionPath, 'node_modules/@polyhobbyist/babylon_ros/dist')));
    paths.push(vscode.Uri.file(path.join(this._context.extensionPath, 'node_modules/babylonjs')));


    // Setup initial content for the webview
    webviewPanel.webview.options = {
      enableScripts: true,
      localResourceRoots: paths
    };
    webviewPanel.webview.html = this.getHtmlForWebview(webviewPanel.webview);

    vscode.workspace.onDidChangeTextDocument(e => {
      if (e.document.uri.toString() === this.uri.toString()) {
        this.refresh();
      }
    }, null, this._disposables);

    webviewPanel.onDidDispose(() => {
        this.dispose();
    }, null, this._context.subscriptions);        


    vscode.workspace.onDidChangeConfiguration(event => {
        this.updateColors();
    }, this, this._context.subscriptions);

    webviewPanel.webview.onDidReceiveMessage(
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
          }
      },
      undefined,
      this._disposables
      );

  }
  
 private refresh() {
    this.updateColors();

    // Handle STL, DAE and other 3D files directly
    this.webviewPanel?.webview.postMessage({
      command: 'view3DFile',
      filename: this.webviewPanel?.webview.asWebviewUri(this.uri).toString()
    });
  }

  private updateColors() {
    if (this.webviewPanel) {
        const config = vscode.workspace.getConfiguration("urdf-editor");
        this.webviewPanel.webview.postMessage({ 
            command: 'colors', 
            cameraRadius: config.get("CameraDistanceToRobot", "1.0"),
            defaultCameraAlpha: config.get("CameraAlpha", -Math.PI / 3),
            defaultCameraBeta: config.get("CameraBeta", 5 * Math.PI / 12),
            defaultCameraRadius: config.get("CameraRadius", 1),
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


private getHtmlForWebview(webview: vscode.Webview): string {
    const webviewUri = util.getUri(webview, this._context.extensionUri, ["dist", "webview.js"]);
    const webviewUriSourceMap = util.getUri(webview, this._context.extensionUri, ["dist", "webview.js.map"]);
    const webviewUriUrdf = util.getUri(webview, this._context.extensionUri, ["node_modules/@polyhobbyist/babylon_ros/dist", "ros.js"]);
    const webviewUriBabylon = util.getUri(webview, this._context.extensionUri, ["node_modules/babylonjs", "babylon.max.js"]);
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

  public dispose() {
    while (this._disposables.length) {
        const disposable = this._disposables.pop();
        if (disposable) {
            disposable.dispose();
        }
    }

    this.webviewPanel?.dispose();
  }

};


