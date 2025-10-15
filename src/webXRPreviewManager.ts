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
    this._app = express();
  }

  public preview(
    resource: vscode.Uri
  ): void {
    this.uri = resource;
    this._trace.appendLine(`previewing ${resource}`);
    this.startServer();
  }

  private startServer() {
    this._app.get('/', async (req, res) => {
      res.send(await this._getWebviewContent());
    });

    // add dist folder to serve static files
    let distPath = this._context.asAbsolutePath("dist");
    this._app.use(express.static(distPath));
    this._app.use(express.static(this._context.asAbsolutePath("node_modules/@polyhobbyist/babylon_ros/dist")));
    this._app.use(express.static(this._context.asAbsolutePath("node_modules/babylonjs")));

    // Create paths from workspace folders and serve them as static content
    var workspaceFolders = vscode.workspace.workspaceFolders;
    if (workspaceFolders) {
        workspaceFolders.forEach((folder) => {
            this._app.use(express.static(folder.uri.path));
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

    try {
      var [urdfText, packagesNotFound] = await util.processXacro(this.uri, (packageName: vscode.Uri) => {
        return packageName.path;
      });


        var previewFile = this.uri.toString();
        return urdfText;
      } catch (err: any) {
        this._trace.appendLine(`Error processing URDF: ${err.message}`);
      }

      return "";
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