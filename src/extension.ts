import * as vscode from 'vscode';
import * as path from 'path';
import URDFPreviewManager from "./previewManager";
import WebXRPreviewManager from "./webXRPreviewManager";
import * as util from "./utils";

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

  // Register custom-urdf-instructions.md for Copilot/GitHub context
  const urdfInstructionsUri = vscode.Uri.file(path.join(context.extensionPath, 'dist', 'prompts', 'custom-urdf-instructions.md'));
  vscode.workspace.getConfiguration('markdown').update('preview.markdown', urdfInstructionsUri, vscode.ConfigurationTarget.Global);

  // Register language support for URDF and XACRO files
  // This is now handled by the package.json configuration

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
  context.subscriptions.push(previewURDFWebXRCommand);

  const exportURDFCommand = vscode.commands.registerCommand("urdf-editor.export", () => {
    if (vscode.window.activeTextEditor) {
      // default filename should be urdf, but with the same name as the current file without xacro if it is there.
      var defaultFilename = vscode.window.activeTextEditor.document.uri.fsPath;
      var ext = path.extname(defaultFilename);
      if (ext === ".xacro") {
        defaultFilename = defaultFilename.slice(0, -6);
      }

      ext = path.extname(defaultFilename);
      if (ext === ".urdf") {
        defaultFilename = defaultFilename.slice(0, -5);
      }

      // Open File save dialog to get file to save to
      vscode.window.showSaveDialog({
        filters: {
          'URDF': ['urdf']
        },
        saveLabel: 'Export URDF',
        title: 'Export URDF',
        defaultUri: vscode.Uri.file(defaultFilename + ".export.urdf"),
      }).then(async (uri) => {
        if (uri) {
          if (vscode.window.activeTextEditor && vscode.window.activeTextEditor.document.uri) {
            // Write the contents of the active editor to the file
            var [urdfText, packagesNotFound] = await util.processXacro(vscode.window.activeTextEditor.document.uri.fsPath, (packageName: vscode.Uri) => {
              return packageName.fsPath;
            });

            var saveBuffer = Buffer.from(urdfText);
            
            await vscode.workspace.fs.writeFile(uri, saveBuffer);
          }
        }
      });
    }
  });

  context.subscriptions.push(exportURDFCommand);
}

export function deactivate() {

}
