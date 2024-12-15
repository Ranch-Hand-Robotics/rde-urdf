import * as vscode from 'vscode';
import URDFPreviewManager from "./previewManager";
import WebXRPreviewManager from "./webXRPreviewManager";

import { Viewer3DProvider } from './3DViewerProvider';

var tracing: vscode.OutputChannel = vscode.window.createOutputChannel("URDF Editor");

var urdfManager: URDFPreviewManager | null = null;
var urdfXRManager: WebXRPreviewManager | null = null;

var viewProvider: Viewer3DProvider | null = null;

export function activate(context: vscode.ExtensionContext) {

  console.log('"urdf-editor" is now active!');
  urdfManager = new URDFPreviewManager(context, tracing);
  urdfXRManager = new WebXRPreviewManager(context, tracing);
  viewProvider = new Viewer3DProvider(context);
  vscode.window.registerWebviewPanelSerializer('urdfPreview_standalone', urdfManager);

  vscode.window.registerCustomEditorProvider('urdf-editor.Viewer3D', viewProvider, 
    {
      webviewOptions: 
      { 
        retainContextWhenHidden: true
      }
    }
  );

  const previewURDFCommand = vscode.commands.registerCommand("urdf-editor.create", () => {
    if (urdfManager && vscode.window.activeTextEditor) {
      urdfManager.preview(vscode.window.activeTextEditor.document.uri);
    }
  });

  context.subscriptions.push(previewURDFCommand);

  const previewURDFWebXRCommand = vscode.commands.registerCommand("urdf-editor.createXR", () => {
    if (urdfXRManager && vscode.window.activeTextEditor) {
      urdfXRManager.preview(vscode.window.activeTextEditor.document.uri);
    }
  });

  context.subscriptions.push(previewURDFCommand);

}


export function deactivate() {

}
