import * as vscode from 'vscode';
import * as path from 'path';
import URDFPreviewManager from "./previewManager";
import WebXRPreviewManager from "./webXRPreviewManager";
import * as util from "./utils";

import { Viewer3DProvider } from './3DViewerProvider';

export var tracing: vscode.OutputChannel = vscode.window.createOutputChannel("URDF Editor");

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

  // Force the workspace to use github.copilot.chat.codeGeneration.useInstructionFiles to true
  vscode.workspace.getConfiguration().update('github.copilot.chat.codeGeneration.useInstructionFiles', true, vscode.ConfigurationTarget.Workspace);

  // Add prompts/urdf-instructions.md to the github.copilot.chat.codeGeneration.instruction array if it is not already there
  var instructions = vscode.workspace.getConfiguration().get('github.copilot.chat.codeGeneration.instruction') as Array<any> || [];
  if (!instructions.includes('urdf-instructions.md')) {
    let extensionInstructionPath = path.join(context.extensionPath, 'prompts/urdf-instructions.md');
    instructions.push({ file: extensionInstructionPath });
    vscode.workspace.getConfiguration().update('github.copilot.chat.codeGeneration.instruction', instructions, vscode.ConfigurationTarget.Workspace);
  }

  // Register language support for URDF and XACRO files
  // This is now handled by the package.json configuration
  const previewURDFCommand = vscode.commands.registerCommand("urdf-editor.create", (uri?: vscode.Uri) => {
    if (urdfManager) {
      // If called from context menu, use the provided URI
      if (uri) {
        urdfManager.preview(uri);
      } 
      // If called from command palette, use active text editor
      else if (vscode.window.activeTextEditor) {
        urdfManager.preview(vscode.window.activeTextEditor.document.uri);
      }
    }
  });

  context.subscriptions.push(previewURDFCommand);
  const previewURDFWebXRCommand = vscode.commands.registerCommand("urdf-editor.createXR", (uri?: vscode.Uri) => {
    if (urdfXRManager) {
      // If called from context menu, use the provided URI
      if (uri) {
        urdfXRManager.preview(uri);
      }
      // If called from command palette, use active text editor
      else if (vscode.window.activeTextEditor) {
        urdfXRManager.preview(vscode.window.activeTextEditor.document.uri);
      }
    }
  });
  context.subscriptions.push(previewURDFWebXRCommand);
  const exportURDFCommand = vscode.commands.registerCommand("urdf-editor.export", (uri?: vscode.Uri) => {
    // Determine which URI to use
    let documentUri: vscode.Uri | undefined;
    if (uri) {
      // Called from context menu
      documentUri = uri;
    } else if (vscode.window.activeTextEditor) {
      // Called from command palette
      documentUri = vscode.window.activeTextEditor.document.uri;
    }

    if (documentUri) {
      // default filename should be urdf, but with the same name as the current file without xacro if it is there.
      var defaultFilename = documentUri.fsPath;
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
      }).then(async (saveUri) => {
        if (saveUri && documentUri) {
          // Write the contents of the file to the export location
          try {
            var [urdfText, packagesNotFound] = await util.processXacro(documentUri.fsPath, (packageName: vscode.Uri) => {
              return packageName.fsPath;
            });

            var saveBuffer = Buffer.from(urdfText);
            
            await vscode.workspace.fs.writeFile(saveUri, saveBuffer);

            if (packagesNotFound.length > 0) {
                var packagesNotFoundList = packagesNotFound.join('\n');
    
                packagesNotFoundList += '\n\nNOTE: This version of the URDF Exporter will not look for packages outside the workspace.';
                vscode.window.showErrorMessage("The following packages were not found in the workspace:\n" + packagesNotFoundList);
            }

          } catch (error) {
            vscode.window.showErrorMessage(`Failed to export URDF: ${error instanceof Error ? error.message : String(error)}`);
          }
        }
      });
    }
  });

  context.subscriptions.push(exportURDFCommand);
}

export function deactivate() {

}
