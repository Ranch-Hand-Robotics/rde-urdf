import { StreamableHTTPServerTransport } from '@modelcontextprotocol/sdk/server/streamableHttp.js';
import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import { 
  ErrorCode,
  McpError,
  isInitializeRequest,
} from '@modelcontextprotocol/sdk/types.js';
import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import * as z from 'zod';
import { tracing } from './extension';
import * as express from 'express';
import { randomUUID } from 'node:crypto';
import {urdfManager} from "./extension";
import { generateOpenSCADLibrariesDocumentation, convertLibrariesDocumentationToMarkdown, validateOpenSCAD } from './openscad';

/**
 * URDF MCP Server Implementation
 * 
 * This MCP server provides screenshot capabilities for URDF, Xacro, and OpenSCAD files.
 * It uses BabylonJS and the babylon_ros library to render 3D scenes and capture screenshots.
 * Includes HTTP transport for client connections and session management.
 */
export class UrdfMcpServer {
  private server: McpServer;
  private app: express.Application;
  private transports: { [sessionId: string]: StreamableHTTPServerTransport } = {};
  private isRunning = false;
  private port: number;
  private httpServer: any;

  constructor(port: number = 3005) {
    this.port = port;
    this.app = express();
    this.app.use(express.json());
    
    this.server = new McpServer(
      {
        name: 'urdf',
        description: 'URDF/Xacro/OpenSCAD MCP Server',
        version: '1.0.0',
      },
      {
        capabilities: {
          tools: {},
        },
      }
    );

    this.setupRoutes();
    this.setupTools();
  }

  private setupTools(): void {
    // Register the screenshot tool using McpServer's tool registration
    this.server.registerTool('take_screenshot', {
        title: 'Returns a Screenshot of the active URDF, Xacro or OpenSCAD File',
        description: 'When you update a URDF, Xacro, or OpenSCAD file, use this to capture an image and verify that the rendered file matches the user\'s expectations. If a filename is provided, will find or open a preview for that file. Otherwise uses the currently active preview.',
        inputSchema: {
          filename: z.string().optional().describe('Optional: The path to a specific file to screenshot (if not provided, uses the currently active preview)'),
          width: z.number().int().optional().describe('Screenshot width in pixels (default: 1024)'),
          height: z.number().int().optional().describe('Screenshot height in pixels (default: 1024)')
        } as any
    }, async (args) => {
      const argsObj = args as any;
      const filename = argsObj.filename as string | undefined;
      const width = (argsObj.width as number) || 1024;
      const height = (argsObj.height as number) || 1024;
      
      try {
        if (!urdfManager) {
          tracing.appendLine(`MCP Server: No URDF manager available`);
          return { 
            content: [{ 
              type: 'text', 
              text: 'I cannot generate a screenshot because the URDF manager is not available. Please restart the extension and try again.'
            }] 
          } as any;
        }

        // Get or create preview using helper method
        const { preview, error } = await this.getOrCreatePreview(filename);
        if (error) {
          return { 
            content: [{ 
              type: 'text', 
              text: error
            }] 
          } as any;
        }

        tracing.appendLine(`MCP Server: Taking screenshot (${width}x${height})`);

        const base64Image = await preview.takeScreenshot(width, height, 30000);

        return {
          content: [
            {
              type: 'image',
              data: base64Image,
              mimeType: 'image/png',
            },
          ],
        } as any;
      } catch (error) {
        tracing.appendLine(`MCP Server error: ${error instanceof Error ? error.message : String(error)}`);
        
        if (error instanceof McpError) {
          throw error;
        }
        
        // Provide helpful error message based on the error type
        let errorMessage = error instanceof Error ? error.message : String(error);
        
        // If timeout occurred, try to validate OpenSCAD files and get specific errors
        if (errorMessage.includes('timed out') && filename) {
          const ext = path.extname(filename).toLowerCase();
          if (ext === '.scad') {
            try {
              const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
              const validationResult = await validateOpenSCAD(filename, undefined, workspaceRoot);
              
              if (!validationResult.valid && validationResult.errors.length > 0) {
                // Return detailed OpenSCAD compilation errors
                let detailedError = `Screenshot timed out because the OpenSCAD file has compilation errors:\n\n`;
                detailedError += '❌ Compilation Errors:\n';
                validationResult.errors.forEach((err, idx) => {
                  detailedError += `${idx + 1}. ${err}\n`;
                });
                
                if (validationResult.warnings.length > 0) {
                  detailedError += '\n⚠ Warnings:\n';
                  validationResult.warnings.forEach((warn, idx) => {
                    detailedError += `${idx + 1}. ${warn}\n`;
                  });
                }
                
                return {
                  content: [{
                    type: 'text',
                    text: detailedError
                  }]
                } as any;
              }
            } catch (validationError) {
              tracing.appendLine(`Failed to validate OpenSCAD after timeout: ${validationError instanceof Error ? validationError.message : String(validationError)}`);
            }
          }
        }
        
        if (errorMessage.includes('timed out')) {
          errorMessage += ' This usually indicates a rendering error in the file. Please check the file for syntax errors or invalid geometry.';
        }
        
        throw new McpError(
          ErrorCode.InternalError,
          `Failed to take screenshot: ${errorMessage}`
        );
      }
    });

    // Register the OpenSCAD libraries documentation tool
    this.server.registerTool('get_openscad_libraries', {
        title: 'Get OpenSCAD Libraries Documentation',
        description: 'Retrieves comprehensive documentation of all available OpenSCAD libraries, including modules, functions, and their parameters. Use this to understand what OpenSCAD utilities are available when creating or updating OpenSCAD files.',
        inputSchema: {}
    }, async (args) => {
      try {
        tracing.appendLine(`MCP Server: Generating OpenSCAD libraries documentation`);

        // Get workspace root
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
        if (!workspaceRoot) {
          return { 
            content: [{ 
              type: 'text', 
              text: 'No workspace folder is open. Please open a workspace to scan for OpenSCAD libraries.'
            }] 
          };
        }

        // Generate documentation
        const documentation = await generateOpenSCADLibrariesDocumentation(workspaceRoot);
        const markdown = convertLibrariesDocumentationToMarkdown(documentation);

        if (documentation.libraries.length === 0) {
          return { 
            content: [{ 
              type: 'text', 
              text: 'No OpenSCAD libraries found in the current workspace. You can add library paths in the extension settings (urdf-editor.OpenSCADLibraryPaths) or place .scad files in the default OS-specific library locations.'
            }] 
          };
        }

        tracing.appendLine(`MCP Server: Found ${documentation.libraries.length} OpenSCAD libraries`);

        return {
          content: [
            {
              type: 'text',
              text: markdown,
            },
          ],
        };
      } catch (error) {
        tracing.appendLine(`MCP Server error generating OpenSCAD docs: ${error instanceof Error ? error.message : String(error)}`);
        
        if (error instanceof McpError) {
          throw error;
        }
        
        throw new McpError(
          ErrorCode.InternalError,
          `Failed to generate OpenSCAD libraries documentation: ${error instanceof Error ? error.message : String(error)}`
        );
      }
    });

    // Register the file screenshot tool (with optional save-to-file capability)
    this.server.registerTool('take_screenshot_of_file', {
        title: 'Take Screenshot of Specific URDF/Xacro/OpenSCAD File',
        description: 'Takes a screenshot of a specific URDF, Xacro, or OpenSCAD file. Returns the image data, or optionally saves it to a file. Will open a preview if one is not already active for the file.',
        inputSchema: {
          filename: z.string().describe('The path to the URDF, Xacro, or OpenSCAD file (absolute or relative to workspace)'),
          saveFilename: z.string().optional().describe('Optional: If provided, saves the screenshot to this file instead of returning image data (e.g., "output.png" or "docs/screenshots/robot.png")'),
          width: z.number().int().optional().describe('Screenshot width in pixels (default: 1024)'),
          height: z.number().int().optional().describe('Screenshot height in pixels (default: 1024)')
        } as any
    }, async (args) => {
      const argsObj = args as any;
      const filename = argsObj.filename as string;
      const saveFilename = argsObj.saveFilename as string | undefined;
      const width = (argsObj.width as number) || 1024;
      const height = (argsObj.height as number) || 1024;
      
      try {
        if (!filename) {
          return { 
            content: [{ 
              type: 'text', 
              text: 'Missing required parameter: filename. Please provide the path to a URDF, Xacro, or OpenSCAD file.'
            }] 
          } as any;
        }

        if (!urdfManager) {
          tracing.appendLine(`MCP Server: No URDF manager available`);
          return { 
            content: [{ 
              type: 'text', 
              text: 'I cannot generate a screenshot because the URDF manager is not available. Please restart the extension and try again.'
            }] 
          } as any;
        }

        // Resolve the file path
        let fileUri: vscode.Uri;
        try {
          if (path.isAbsolute(filename)) {
            fileUri = vscode.Uri.file(filename);
          } else {
            // Try to resolve relative to workspace
            const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
            if (!workspaceRoot) {
              return { 
                content: [{ 
                  type: 'text', 
                  text: `Cannot resolve relative path "${filename}" - no workspace folder is open.`
                }] 
              } as any;
            }
            fileUri = vscode.Uri.file(path.join(workspaceRoot, filename));
          }
        } catch (error) {
          return { 
            content: [{ 
              type: 'text', 
              text: `Invalid filename: "${filename}". Please provide a valid file path.`
            }] 
          } as any;
        }

        // Check if the file exists
        try {
          await vscode.workspace.fs.stat(fileUri);
        } catch (error) {
          return { 
            content: [{ 
              type: 'text', 
              text: `File not found: "${fileUri.fsPath}". Please check the path and try again.`
            }] 
          } as any;
        }

        // Check if it's a supported file type
        const ext = path.extname(fileUri.fsPath).toLowerCase();
        if (ext !== '.urdf' && ext !== '.xacro' && ext !== '.scad') {
          return { 
            content: [{ 
              type: 'text', 
              text: `Unsupported file type: "${ext}". Only .urdf, .xacro, and .scad files are supported.`
            }] 
          } as any;
        }

        tracing.appendLine(`MCP Server: Taking screenshot of ${fileUri.fsPath} (${width}x${height})`);

        // Check if there's already an existing preview for this file
        let preview = urdfManager.getExistingPreview(fileUri);
        
        if (!preview) {
          // No existing preview, create a new one
          tracing.appendLine(`MCP Server: Creating new preview for ${fileUri.fsPath}`);
          urdfManager.preview(fileUri);
          
          // Get the newly created preview
          preview = urdfManager.activePreview;
          
          if (!preview) {
            return { 
              content: [{ 
                type: 'text', 
                text: `Failed to create preview for "${fileUri.fsPath}". The file may have errors or be unsupported.`
              }] 
            } as any;
          }

          // Wait a bit for the preview to load
          await new Promise(resolve => setTimeout(resolve, 2000));
        } else {
          tracing.appendLine(`MCP Server: Using existing preview for ${fileUri.fsPath}`);
          // Make sure the existing preview is revealed/active
          preview.reveal();
          // Wait a bit to ensure it's fully loaded
          await new Promise(resolve => setTimeout(resolve,2000));
        }

        // Take the screenshot
        const base64Image = await preview.takeScreenshot(width, height, 30000);

        tracing.appendLine(`MCP Server: Successfully captured screenshot of ${fileUri.fsPath}`);

        // If saveFilename is provided, save to file
        if (saveFilename) {
          const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
          let savePath: string;
          
          if (path.isAbsolute(saveFilename)) {
            savePath = saveFilename;
          } else if (workspaceRoot) {
            savePath = path.join(workspaceRoot, saveFilename);
          } else {
            return { 
              content: [{ 
                type: 'text', 
                text: 'Cannot determine save location. Please provide an absolute path or open a workspace.'
              }] 
            } as any;
          }

          // Ensure directory exists
          const saveDir = path.dirname(savePath);
          if (!fs.existsSync(saveDir)) {
            fs.mkdirSync(saveDir, { recursive: true });
          }

          // Check if file already exists (for logging purposes)
          const fileExists = fs.existsSync(savePath);

          // Decode base64 and write to file (overwrites if exists)
          const buffer = Buffer.from(base64Image, 'base64');
          fs.writeFileSync(savePath, buffer);

          const action = fileExists ? 'overwritten' : 'created';
          tracing.appendLine(`MCP Server: Screenshot ${action} at ${savePath}`);

          return {
            content: [
              {
                type: 'text',
                text: `Screenshot saved successfully to: ${savePath}`,
              },
            ],
          } as any;
        }

        // Return image data
        return {
          content: [
            {
              type: 'image',
              data: base64Image,
              mimeType: 'image/png',
            },
          ],
        } as any;
      } catch (error) {
        tracing.appendLine(`MCP Server error taking screenshot: ${error instanceof Error ? error.message : String(error)}`);
        
        if (error instanceof McpError) {
          throw error;
        }
        
        // Provide helpful error message based on the error type
        let errorMessage = error instanceof Error ? error.message : String(error);
        
        // If timeout occurred on an OpenSCAD file, try to validate and get specific errors
        if (errorMessage.includes('timed out') && filename) {
          const ext = path.extname(filename).toLowerCase();
          if (ext === '.scad') {
            try {
              const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
              const validationResult = await validateOpenSCAD(filename, undefined, workspaceRoot);
              
              if (!validationResult.valid && validationResult.errors.length > 0) {
                // Return detailed OpenSCAD compilation errors
                let detailedError = `Screenshot timed out because the OpenSCAD file has compilation errors:\n\n`;
                detailedError += '❌ Compilation Errors:\n';
                validationResult.errors.forEach((err, idx) => {
                  detailedError += `${idx + 1}. ${err}\n`;
                });
                
                if (validationResult.warnings.length > 0) {
                  detailedError += '\n⚠ Warnings:\n';
                  validationResult.warnings.forEach((warn, idx) => {
                    detailedError += `${idx + 1}. ${warn}\n`;
                  });
                }
                
                return {
                  content: [{
                    type: 'text',
                    text: detailedError
                  }]
                } as any;
              }
            } catch (validationError) {
              tracing.appendLine(`Failed to validate OpenSCAD after timeout: ${validationError instanceof Error ? validationError.message : String(validationError)}`);
            }
          }
          errorMessage += ' This usually indicates a rendering error in the file. Please check the file for syntax errors or invalid geometry.';
        }
        
        if (errorMessage.includes('EACCES') || errorMessage.includes('permission')) {
          errorMessage = 'Permission denied. Check that you have write access to the target directory.';
        }
        
        throw new McpError(
          ErrorCode.InternalError,
          `Failed to take screenshot: ${errorMessage}`
        );
      }
    });

    // Register the OpenSCAD validation tool
    this.server.registerTool('validate_openscad', {
        title: 'Validate OpenSCAD File for Compilation Errors',
        description: 'Validates an OpenSCAD file by attempting to compile it and returns any syntax or compilation errors. Use this to check if your OpenSCAD code is valid before declaring completion. Parameters: filename (required - path to .scad file), content (optional - file content to validate instead of reading from file).',
        inputSchema: {}
    }, async (args) => {
      try {
        const argsObj = args as any;
        const filename = argsObj.filename as string;
        const content = argsObj.content as string | undefined;

        if (!filename) {
          return { 
            content: [{ 
              type: 'text', 
              text: 'Missing required parameter: filename. Please provide the path to an OpenSCAD (.scad) file.'
            }] 
          } as any;
        }

        tracing.appendLine(`MCP Server: Validating OpenSCAD file: ${filename}`);

        // Resolve the file path
        let fileUri: vscode.Uri;
        try {
          if (path.isAbsolute(filename)) {
            fileUri = vscode.Uri.file(filename);
          } else {
            // Try to resolve relative to workspace
            const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
            if (!workspaceRoot) {
              return { 
                content: [{ 
                  type: 'text', 
                  text: `Cannot resolve relative path "${filename}" - no workspace folder is open.`
                }] 
              } as any;
            }
            fileUri = vscode.Uri.file(path.join(workspaceRoot, filename));
          }
        } catch (error) {
          return { 
            content: [{ 
              type: 'text', 
              text: `Invalid filename: "${filename}". Please provide a valid file path.`
            }] 
          } as any;
        }

        // Check if the file exists (unless content is provided)
        if (!content) {
          try {
            await vscode.workspace.fs.stat(fileUri);
          } catch (error) {
            return { 
              content: [{ 
                type: 'text', 
                text: `File not found: "${fileUri.fsPath}". Please check the path and try again.`
              }] 
            } as any;
          }
        }

        // Check if it's a .scad file
        const ext = path.extname(fileUri.fsPath).toLowerCase();
        if (ext !== '.scad') {
          return { 
            content: [{ 
              type: 'text', 
              text: `File must be an OpenSCAD file (.scad), got: "${ext}".`
            }] 
          } as any;
        }

        // Get workspace root for library resolution
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;

        // Validate the OpenSCAD file
        const result = await validateOpenSCAD(fileUri.fsPath, content, workspaceRoot);

        tracing.appendLine(`MCP Server: Validation complete. Valid: ${result.valid}, Errors: ${result.errors.length}, Warnings: ${result.warnings.length}`);

        // Format the response
        let responseText = '';
        
        if (result.valid) {
          responseText = `✓ OpenSCAD file is valid and compiles successfully.\n\nFile: ${fileUri.fsPath}`;
          
          if (result.warnings.length > 0) {
            responseText += '\n\n⚠ Warnings:\n';
            result.warnings.forEach((warning, idx) => {
              responseText += `${idx + 1}. ${warning}\n`;
            });
          }
        } else {
          responseText = `✗ OpenSCAD file has compilation errors.\n\nFile: ${fileUri.fsPath}\n\n`;
          responseText += '❌ Errors:\n';
          result.errors.forEach((error, idx) => {
            responseText += `${idx + 1}. ${error}\n`;
          });
          
          if (result.warnings.length > 0) {
            responseText += '\n⚠ Warnings:\n';
            result.warnings.forEach((warning, idx) => {
              responseText += `${idx + 1}. ${warning}\n`;
            });
          }
        }

        return {
          content: [
            {
              type: 'text',
              text: responseText,
            },
          ],
        } as any;
      } catch (error) {
        tracing.appendLine(`MCP Server error validating OpenSCAD: ${error instanceof Error ? error.message : String(error)}`);
        
        if (error instanceof McpError) {
          throw error;
        }
        
        // Provide helpful error message based on the error type
        let errorMessage = error instanceof Error ? error.message : String(error);
        if (errorMessage.includes('timed out')) {
          errorMessage += ' This usually indicates a rendering error in the file. Please check the file for syntax errors or invalid geometry.';
        }
        
        throw new McpError(
          ErrorCode.InternalError,
          `Failed to take screenshot: ${errorMessage}`
        );
      }
    });
  }

  /**
   * Helper method to get or create a preview for a given file
   * @param filename Optional file path. If not provided, returns the active preview.
   * @returns Object with either preview or error
   */
  private async getOrCreatePreview(filename?: string): Promise<{preview?: any; error?: string}> {
    if (!urdfManager) {
      return { error: 'No URDF manager available. Please restart the extension and try again.' };
    }

    // If no filename provided, use active preview
    if (!filename) {
      const activePreview = urdfManager.activePreview;
      if (!activePreview) {
        return { error: 'No active preview available. Please open a preview first or provide a filename parameter.' };
      }
      return { preview: activePreview };
    }

    // Resolve the file path
    let fileUri: vscode.Uri;
    try {
      if (path.isAbsolute(filename)) {
        fileUri = vscode.Uri.file(filename);
      } else {
        const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
        if (!workspaceRoot) {
          return { error: `Cannot resolve relative path "${filename}" - no workspace folder is open.` };
        }
        fileUri = vscode.Uri.file(path.join(workspaceRoot, filename));
      }
    } catch (error) {
      return { error: `Invalid filename: "${filename}". Please provide a valid file path.` };
    }

    // Check if the file exists
    try {
      await vscode.workspace.fs.stat(fileUri);
    } catch (error) {
      return { error: `File not found: "${fileUri.fsPath}". Please check the path and try again.` };
    }

    // Check if it's a supported file type
    const ext = path.extname(fileUri.fsPath).toLowerCase();
    if (ext !== '.urdf' && ext !== '.xacro' && ext !== '.scad') {
      return { error: `Unsupported file type: "${ext}". Only .urdf, .xacro, and .scad files are supported.` };
    }

    // Check if there's already an existing preview for this file
    let preview = urdfManager.getExistingPreview(fileUri);
    
    if (!preview) {
      // No existing preview, create a new one
      tracing.appendLine(`MCP Server: Creating new preview for ${fileUri.fsPath}`);
      urdfManager.preview(fileUri);
      
      // Get the newly created preview
      preview = urdfManager.activePreview;
      
      if (!preview) {
        return { error: `Failed to create preview for "${fileUri.fsPath}". The file may have errors or be unsupported.` };
      }

      // Wait a bit for the preview to load
      await new Promise(resolve => setTimeout(resolve, 2000));
    } else {
      tracing.appendLine(`MCP Server: Using existing preview for ${fileUri.fsPath}`);
      // Make sure the existing preview is revealed/active
      preview.reveal();
      // Wait a bit to ensure it's fully loaded
      await new Promise(resolve => setTimeout(resolve, 500));
    }

    return { preview };
  }

  private setupRoutes(): void {
    // Handle POST requests for client-to-server communication
    this.app.post('/mcp', async (req, res) => {
      // Check for existing session ID
      const sessionId = req.headers['mcp-session-id'] as string | undefined;
      let transport: StreamableHTTPServerTransport;

      if (sessionId && this.transports[sessionId]) {
        // Reuse existing transport
        transport = this.transports[sessionId];
      } else if (!sessionId && isInitializeRequest(req.body)) {
        // New initialization request
        transport = new StreamableHTTPServerTransport({
          sessionIdGenerator: () => randomUUID(),
          onsessioninitialized: (sessionId) => {
            // Store the transport by session ID
            this.transports[sessionId] = transport;
          },
          // DNS rebinding protection is disabled by default for backwards compatibility
          // enableDnsRebindingProtection: true,
          // allowedHosts: ['127.0.0.1'],
        });

        // Clean up transport when closed
        transport.onclose = () => {
          if (transport.sessionId) {
            delete this.transports[transport.sessionId];
          }
        };

        // Connect to the MCP server
        await this.server.connect(transport);
      } else {
        // Invalid request
        res.status(400).json({
          jsonrpc: '2.0',
          error: {
            code: -32000,
            message: 'Bad Request: No valid session ID provided',
          },
          id: null,
        });
        return;
      }

      // Handle the request
      await transport.handleRequest(req, res, req.body);
    });

    // Reusable handler for GET and DELETE requests
    const handleSessionRequest = async (req: express.Request, res: express.Response) => {
      const sessionId = req.headers['mcp-session-id'] as string | undefined;
      if (!sessionId || !this.transports[sessionId]) {
        res.status(400).send('Invalid or missing session ID');
        return;
      }
      
      const transport = this.transports[sessionId];
      await transport.handleRequest(req, res);
    };

    // Handle GET requests for server-to-client notifications via SSE
    this.app.get('/mcp', handleSessionRequest);

    // Handle DELETE requests for session termination
    this.app.delete('/mcp', handleSessionRequest);
  }

  /**
   * Starts the MCP server
   */
  async start(): Promise<void> {
    if (this.isRunning) {
      throw new Error('MCP server is already running');
    }

    try {
      // Start the Express HTTP server
      this.httpServer = this.app.listen(this.port, () => {
        this.isRunning = true;
        tracing.appendLine(`URDF MCP Server started on port ${this.port}`);
      });
    } catch (error) {
      const message = `Failed to start MCP server: ${error instanceof Error ? error.message : String(error)}`;
      tracing.appendLine(message);
      throw new Error(message);
    }
  }

  /**
   * Stops the MCP server
   */
  async stop(): Promise<void> {
    if (!this.isRunning) {
      return;
    }

    try {
      // Close all active transports
      for (const sessionId in this.transports) {
        const transport = this.transports[sessionId];
        if (transport) {
          await transport.close();
        }
      }
      this.transports = {};

      // Stop the HTTP server
      if (this.httpServer) {
        await new Promise<void>((resolve, reject) => {
          this.httpServer.close((err: any) => {
            if (err) {
              reject(err);
            } else {
              resolve();
            }
          });
        });
        this.httpServer = null;
      }
      
      this.isRunning = false;
      tracing.appendLine('URDF MCP Server stopped');
    } catch (error) {
      const message = `Failed to stop MCP server: ${error instanceof Error ? error.message : String(error)}`;
      tracing.appendLine(message);
      throw new Error(message);
    }
  }

  /**
   * Gets the current status of the server
   */
  getStatus(): { isRunning: boolean; port: number } {
    return {
      isRunning: this.isRunning,
      port: this.port,
    };
  }
}
