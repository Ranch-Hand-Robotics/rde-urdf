import * as vscode from 'vscode';
import * as path from 'path';
import URDFPreviewManager from "./previewManager";
import WebXRPreviewManager from "./webXRPreviewManager";
import * as util from "./utils";
import { WindowMcpConnection } from './mcpConnection';
import { generateAndSaveLibrariesDocumentation } from './openscad';
import { OpenSCADCompletionProvider, OpenSCADHoverProvider } from './openscadCompletion';
import { URDFXacroCompletionProvider, URDFXacroHoverProvider } from './urdfXacroCompletion';
import { OpenSCADDefinitionProvider } from './openscadDefinitionProvider';
import { URDFDefinitionProvider } from './urdfDefinitionProvider';
import { Viewer3DProvider } from './3DViewerProvider';
import * as agents from './agents';
import { registerOpenSCADExportCommands } from './openscadExport';

export var tracing: vscode.OutputChannel = vscode.window.createOutputChannel("URDF Editor");

// Initialize agents module with tracing
agents.setTracing(tracing);

export var urdfManager: URDFPreviewManager | null = null;
export var urdfXRManager: WebXRPreviewManager | null = null;
var viewProvider: Viewer3DProvider | null = null;
var mcpConnection: WindowMcpConnection | undefined;

export async function activate(context: vscode.ExtensionContext) {

  console.log('"urdf-editor" is now active!');
  
  // Initialize agents module with context
  agents.setExtensionContext(context);
  
  urdfManager = new URDFPreviewManager(context, tracing);
  urdfXRManager = new WebXRPreviewManager(context, tracing);
  viewProvider = new Viewer3DProvider(context, tracing);
  vscode.window.registerWebviewPanelSerializer('urdfPreview_standalone', urdfManager);

  // This activation belongs to one VS Code window, even when another window
  // has the same workspace open. Publish only this window's live connection.
  mcpConnection = new WindowMcpConnection(tracing);
  mcpConnection.register(context);
  void mcpConnection.start().catch(error => {
    tracing.appendLine(`Failed to start window-bound MCP server: ${error instanceof Error ? error.message : String(error)}`);
  });

  // Register OpenSCAD IntelliSense completion provider
  const openscadCompletionProvider = vscode.languages.registerCompletionItemProvider(
    { language: 'openscad', scheme: 'file' },
    new OpenSCADCompletionProvider(),
    '(', '"', ' ' // Trigger characters
  );
  context.subscriptions.push(openscadCompletionProvider);

  // Register OpenSCAD hover provider
  const openscadHoverProvider = vscode.languages.registerHoverProvider(
    { language: 'openscad', scheme: 'file' },
    new OpenSCADHoverProvider()
  );
  context.subscriptions.push(openscadHoverProvider);

  // Register OpenSCAD definition provider (Go to Definition / F12)
  const openscadDefinitionProvider = vscode.languages.registerDefinitionProvider(
    { language: 'openscad', scheme: 'file' },
    new OpenSCADDefinitionProvider(tracing)
  );
  context.subscriptions.push(openscadDefinitionProvider);

  // Register URDF/Xacro IntelliSense completion providers
  const urdfCompletionProvider = vscode.languages.registerCompletionItemProvider(
    [
      { scheme: 'file', pattern: '**/*.urdf' },
      { scheme: 'file', pattern: '**/*.xacro' },
      { language: 'xml', scheme: 'file' }
    ],
    new URDFXacroCompletionProvider(),
    '<', '"', '{' // Trigger characters
  );
  context.subscriptions.push(urdfCompletionProvider);

  // Register hover provider for URDF/Xacro
  const urdfHoverProvider = vscode.languages.registerHoverProvider(
    [
      { scheme: 'file', pattern: '**/*.urdf' },
      { scheme: 'file', pattern: '**/*.xacro' },
      { language: 'xml', scheme: 'file' }
    ],
    new URDFXacroHoverProvider()
  );
  context.subscriptions.push(urdfHoverProvider);

  // Register definition provider for URDF/Xacro (Go to Definition / F12)
  const urdfDefinitionProvider = vscode.languages.registerDefinitionProvider(
    [
      { scheme: 'file', pattern: '**/*.urdf' },
      { scheme: 'file', pattern: '**/*.xacro' },
      { language: 'xml', scheme: 'file' }
    ],
    new URDFDefinitionProvider(tracing)
  );
  context.subscriptions.push(urdfDefinitionProvider);

  // Watch for file saves to refresh all previews when dependent files change
  const saveWatcher = vscode.workspace.onDidSaveTextDocument((document) => {
    const ext = path.extname(document.uri.fsPath);
    
    // Check if the saved file is a URDF, Xacro, or OpenSCAD file
    if (ext === '.urdf' || ext === '.xacro' || ext === '.scad') {
      tracing.appendLine(`File saved: ${document.uri.fsPath}, refreshing all previews`);
      
      // Refresh all open previews since files can depend on each other
      if (urdfManager) {
        urdfManager.refresh();
      }
      if (urdfXRManager) {
        urdfXRManager.refresh();
      }
    }
  });
  context.subscriptions.push(saveWatcher);

  vscode.window.registerCustomEditorProvider('urdf-editor.Viewer3D', viewProvider, 
    {
      webviewOptions: 
      { 
        retainContextWhenHidden: true
      }
    }
  );

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

  // Register OpenSCAD export commands (STL, SVG, and Parts)
  registerOpenSCADExportCommands(context, tracing);

  const takeScreenshotCommand = vscode.commands.registerCommand("urdf-editor.takeScreenshot", async (uri?: vscode.Uri) => {
    if (!urdfManager) {
      vscode.window.showErrorMessage('URDF preview manager not available');
      return;
    }

    // Get the active preview
    const preview = urdfManager.activePreview;
    if (!preview) {
      vscode.window.showErrorMessage('No active URDF preview found. Please open a preview first.');
      return;
    }

    try {
      // Take screenshot with default dimensions
      const base64Image = await preview.takeScreenshot(1024, 1024);
      
      // Determine default filename based on the preview resource
      let defaultFilename = 'screenshot';
      if (preview.resource) {
        const basename = path.basename(preview.resource.fsPath);
        const nameWithoutExt = basename.split('.').slice(0, -1).join('.');
        const directory = path.dirname(preview.resource.fsPath);
        defaultFilename = path.join(directory, `${nameWithoutExt}_screenshot`);
      }

      // Show save dialog
      const saveUri = await vscode.window.showSaveDialog({
        filters: {
          'PNG Images': ['png']
        },
        saveLabel: 'Save Screenshot',
        title: 'Save Screenshot as PNG',
        defaultUri: vscode.Uri.file(defaultFilename + '.png'),
      });

      if (saveUri) {
        // Convert base64 to buffer and save
        const imageBuffer = Buffer.from(base64Image, 'base64');
        await vscode.workspace.fs.writeFile(saveUri, imageBuffer);
        
        vscode.window.showInformationMessage(`Screenshot saved to ${saveUri.fsPath}`);
      }
    } catch (error) {
      const message = `Failed to take screenshot: ${error instanceof Error ? error.message : String(error)}`;
      vscode.window.showErrorMessage(message);
      tracing.appendLine(message);
    }
  });

  const generateOpenSCADDocsCommand = vscode.commands.registerCommand("urdf-editor.generateOpenSCADDocs", async () => {
    try {
      // Get workspace root
      const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
      if (!workspaceRoot) {
        vscode.window.showErrorMessage('No workspace folder open');
        return;
      }

      // Show save dialog for output location
      const defaultPath = path.join(workspaceRoot, 'openscad-libraries.md');
      const saveUri = await vscode.window.showSaveDialog({
        filters: {
          'Markdown Files': ['md']
        },
        saveLabel: 'Save OpenSCAD Documentation',
        title: 'Save OpenSCAD Libraries Documentation',
        defaultUri: vscode.Uri.file(defaultPath),
      });

      if (!saveUri) {
        return; // User cancelled
      }

      // Show progress indicator
      await vscode.window.withProgress({
        location: vscode.ProgressLocation.Notification,
        title: "Generating OpenSCAD documentation...",
        cancellable: false
      }, async (progress) => {
        progress.report({ increment: 0, message: "Scanning libraries..." });
        
        await generateAndSaveLibrariesDocumentation(saveUri.fsPath, workspaceRoot);
        
        progress.report({ increment: 100, message: "Documentation generated!" });
      });

      // Open the generated file
      const doc = await vscode.workspace.openTextDocument(saveUri);
      await vscode.window.showTextDocument(doc);
      
      vscode.window.showInformationMessage(`OpenSCAD libraries documentation saved to ${saveUri.fsPath}`);
      tracing.appendLine(`Generated OpenSCAD documentation: ${saveUri.fsPath}`);
      
    } catch (error) {
      const message = `Failed to generate OpenSCAD documentation: ${error instanceof Error ? error.message : String(error)}`;
      vscode.window.showErrorMessage(message);
      tracing.appendLine(message);
    }
  });

  context.subscriptions.push(takeScreenshotCommand);
  context.subscriptions.push(generateOpenSCADDocsCommand);
}

export async function deactivate() {
  // Stop MCP server when extension deactivates
  await mcpConnection?.dispose();
  mcpConnection = undefined;
  
  // Dispose of preview managers
  if (urdfManager) {
    await urdfManager.dispose();
  }
}
