// This file implements an express web server which renders a URDF using Babylon_ROS and serves it to the client using a dev tunnel.

import * as vscode from 'vscode';
import * as express from 'express';
import * as path from 'path';
import { createServer } from 'http';
import * as util from './utils';

export default class WebXRPreviewManager {
  private _context: vscode.ExtensionContext;
  private _trace: vscode.OutputChannel;

  private _app: express.Application;
  private _server: any;
  private _port: number;
  private uri: vscode.Uri | undefined;
  private urdfText: string = "";

  constructor(context, trace) {
    this._context = context;
    this._trace = trace;
    this._port = 3000;
  }

  public preview(
    resource: vscode.Uri
  ): void {
    this.uri = resource;
    this._trace.appendLine(`previewing ${resource}`);
    this.startServer();
  }

  private startServer() {
    this._app = express();
    this._app.get('/', async (req, res) => {
      res.send(await this._getWebviewContent());
    });

    // add dist folder to serve static files
    let distPath = this._context.asAbsolutePath("dist");
    this._app.use(express.static(distPath));
    this._app.use(express.static(this._context.asAbsolutePath("node_modules/@polyhobbyist/babylon_ros/dist")));
    this._app.use(express.static(this._context.asAbsolutePath("node_modules/babylonjs")));

    var paths : vscode.Uri[] = [];

    // Create paths from workspace folders to vscode uri, and add extensions
    var workspaceFolders = vscode.workspace.workspaceFolders;
    if (workspaceFolders) {
        workspaceFolders.forEach((folder) => {
            paths.push(this._app.use(express.static(folder.uri.fsPath)));
        });
    }



    this._server = createServer(this._app);
    this._server.listen(this._port, () => {
      this._trace.appendLine(`Server listening on port ${this._port}`);
    });
  }

  private async _getURDFText() : Promise<string> {
    if (!this.uri) {
      return "";
    }

    var packagesNotFound : any = [];
    var urdfText = "";
    try {
        urdfText = await util.xacro(this.uri.fsPath);

        var packageMap = await util.getPackages();
        if (packageMap !== null) {
            // replace package://(x) with fully resolved paths
            var pattern =  /package:\/\/(.*?)\//g;
            var match;
            while (match = pattern.exec(urdfText)) {
                if (packageMap.hasOwnProperty(match[1]) === false) {
                    if (packagesNotFound.indexOf(match[1]) === -1) {
                        this._trace.appendLine(`Package ${match[1]} not found in workspace.`);
                        packagesNotFound.push(match[1]);
                    }
                } else {
                    var packagePath = await packageMap[match[1]];
                    if (packagePath.charAt(0)  === '/') {
                        // inside of mesh re \source, the loader attempts to concatinate the base uri with the new path. It first checks to see if the
                        // base path has a /, if not it adds it.
                        // We are attempting to use a protocol handler as the base path - which causes this to fail.
                        // basepath - vscode-webview-resource:
                        // full path - /home/test/ros
                        // vscode-webview-resource://home/test/ros.
                        // It should be vscode-webview-resource:/home/test/ros.
                        // So remove the first char.

                        packagePath = packagePath.substr(1);
                    }
                    let normPath = path.normalize(packagePath);
                    let vsPath = vscode.Uri.file(normPath);
                    let newUri = vsPath.fsPath;

                    // find the path relative to the workspace folders
                    var relativePath = "";
                    var workspaceFolders = vscode.workspace.workspaceFolders;
                    if (workspaceFolders) {
                        workspaceFolders.forEach((folder) => {
                            if (newUri.startsWith(folder.uri.fsPath)) {
                                relativePath = newUri.substring(folder.uri.fsPath.length + 1);
                            }
                        });
                    }

                    // replace \ with /
                    relativePath = relativePath.replace(/\\/g, '/');

                    urdfText = urdfText.replace('package://' + match[1], relativePath);
                }
            }
        }

        var previewFile = this.uri.toString();
      } catch (err: any) {
        this._trace.appendLine(`Error processing URDF: ${err.message}`);
      }
      return urdfText;
  }    

  private async _getWebviewContent(): Promise<string> {
    const nonce = util.getNonce();

    this.urdfText = await this._getURDFText();

    // Tip: Install the es6-string-html VS Code extension to enable code highlighting below
    return /*html*/ `
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
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
            <script nonce="${nonce}" src="babylon.max.js"></script>
            <script nonce="${nonce}" src="webview.js"></script>
            <script nonce="${nonce}">
            var urdfText = \`${this.urdfText}\`;

            // after 2 seconds send the URDF to the webview
            setTimeout(() => {
                window.dispatchEvent(new MessageEvent('message', { data: {command: "urdf", urdf: urdfText} }));
            }, 2000);
            </script>
            </body>
        </html>
    `;
}

}