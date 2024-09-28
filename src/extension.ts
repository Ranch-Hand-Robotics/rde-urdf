import * as vscode from 'vscode';
import URDFPreviewManager from "./previewManager";

var tracing: vscode.OutputChannel = vscode.window.createOutputChannel("URDF Editor");

var urdfManager: URDFPreviewManager | null = null;

export function activate(context: vscode.ExtensionContext) {

  console.log('"urdf-editor" is now active!');

  const previewURDFCommand = vscode.commands.registerCommand("urdf-editor.create", () => {
    urdfManager = new URDFPreviewManager(context, tracing);
    vscode.window.registerWebviewPanelSerializer('urdfPreview', urdfManager);
  });

  context.subscriptions.push(previewURDFCommand);
}


export function deactivate() {

}
