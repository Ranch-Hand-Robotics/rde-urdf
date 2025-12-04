import * as vscode from 'vscode';
import * as path from 'path';
import URDFPreviewManager from "./previewManager";
import WebXRPreviewManager from "./webXRPreviewManager";
import * as util from "./utils";
import { UrdfMcpServer } from './mcp';
import { generateAndSaveLibrariesDocumentation } from './openscad';
import { OpenSCADCompletionProvider, OpenSCADHoverProvider } from './openscadCompletion';
import { URDFXacroCompletionProvider, URDFXacroHoverProvider } from './urdfXacroCompletion';
import { OpenSCADDefinitionProvider } from './openscadDefinitionProvider';
import { URDFDefinitionProvider } from './urdfDefinitionProvider';

import { Viewer3DProvider } from './3DViewerProvider';

export var tracing: vscode.OutputChannel = vscode.window.createOutputChannel("URDF Editor");

export var urdfManager: URDFPreviewManager | null = null;
export var urdfXRManager: WebXRPreviewManager | null = null;
var viewProvider: Viewer3DProvider | null = null;
var mcpServer: UrdfMcpServer | null = null;

async function startMcpServer(context: vscode.ExtensionContext): Promise<void> {
  if (mcpServer && mcpServer.getStatus().isRunning) {
    return; // Already running
  }

  try {
    const config = vscode.workspace.getConfiguration('urdf-editor');
    const port = config.get<number>('mcpServerPort', 3005);
    
    mcpServer = new UrdfMcpServer(port);
    await mcpServer.start();
    tracing.appendLine('MCP Server started automatically with first preview');

    // Check if the MCP API is available (only in VS Code, not Cursor)
    if ('lm' in vscode && vscode.lm && 'registerMcpServerDefinitionProvider' in vscode.lm) {
      try {
        // Use type assertion to handle the API that might not be available in all environments
        const lm = vscode.lm as any;
        context.subscriptions.push(lm.registerMcpServerDefinitionProvider('URDF', {
          provideMcpServerDefinitions: async () => {
            let output: any[] = [];

            // Use the configured port for the MCP server
            // Note: McpHttpServerDefinition might not be available in all environments
            if ('McpHttpServerDefinition' in vscode) {
              const McpHttpServerDefinition = (vscode as any).McpHttpServerDefinition;
              output.push( 
                new McpHttpServerDefinition(
                  "URDF",
                  vscode.Uri.parse(`http://localhost:${port}/mcp`)
                )
              );
            }

            return output;
          }
        }));
      } catch (error) {
        tracing.appendLine(`Failed to register MCP server definition provider: ${error instanceof Error ? error.message : String(error)}`);
      }
    }
  } catch (error) {
    tracing.appendLine(`Failed to auto-start MCP server: ${error instanceof Error ? error.message : String(error)}`);
  }
}

async function stopMcpServer(): Promise<void> {
  if (!mcpServer || !mcpServer.getStatus().isRunning) {
    return; // Not running
  }

  try {
    await mcpServer.stop();
    mcpServer = null;
    tracing.appendLine('MCP Server stopped automatically with last preview');
  } catch (error) {
    tracing.appendLine(`Failed to auto-stop MCP server: ${error instanceof Error ? error.message : String(error)}`);
  }
}

export function activate(context: vscode.ExtensionContext) {

  console.log('"urdf-editor" is now active!');
  urdfManager = new URDFPreviewManager(context, tracing);
  urdfXRManager = new WebXRPreviewManager(context, tracing);
  viewProvider = new Viewer3DProvider(context, tracing);
  vscode.window.registerWebviewPanelSerializer('urdfPreview_standalone', urdfManager);

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

  // Set up MCP server lifecycle callbacks for the preview manager
  urdfManager.setMcpServerCallbacks({
    onStartServer: () => startMcpServer(context),
    onStopServer: () => stopMcpServer()
  });

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
  // Dispose of preview managers which will clean up MCP servers
  if (urdfManager) {
    await urdfManager.dispose();
  }
}
