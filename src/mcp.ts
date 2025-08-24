import { StreamableHTTPServerTransport } from '@modelcontextprotocol/sdk/server/streamableHttp.js';
import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import { 
  ErrorCode,
  McpError,
  isInitializeRequest,
} from '@modelcontextprotocol/sdk/types.js';
import * as vscode from 'vscode';
import * as path from 'path';
import { tracing } from './extension';
import * as express from 'express';
import { randomUUID } from 'node:crypto';
import {urdfManager} from "./extension";
import { generateOpenSCADLibrariesDocumentation, convertLibrariesDocumentationToMarkdown } from './openscad';

/**
 * URDF MCP Server Implementation
 * 
 * This MCP server provides screenshot capabilities for URDF, Xacro, and OpenSCAD files.
 * It uses BabylonJS and the babylon_ros library to render 3D scenes and capture screenshots.
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
        description: 'When you update a URDF, Xacro, or OpenSCAD file, use this to capture an image and verify that the rendered file matches the user\'s expectations.',
        inputSchema: {}
    }, async (args) => {
      try {
        const width = 1024;
        const height = 1024;

        if (!urdfManager) {
          tracing.appendLine(`MCP Server: No URDF manager available`);
          return { 
            content: [{ 
              type: 'text', 
              text: 'I cannot generate a screenshot because the URDF manager is not available. Please restart the extension and try again.'
            }] 
          };
        }

        let preview = urdfManager.activePreview;
        if (!preview) {
          tracing.appendLine(`MCP Server: No active preview found for screenshot`);
          return { 
            content: [{ 
              type: 'text', 
              text: 'I cannot generate a screenshot without an active preview. Ask the user to open a preview and try again.'
            }] 
          };
        }

        tracing.appendLine(`MCP Server: Taking screenshot of active preview (${width}x${height})`);

        const base64Image = await preview.takeScreenshot(width, height);

        return {
          content: [
            {
              type: 'image',
              data: base64Image,
              mimeType: 'image/png',
            },
          ],
        };
      } catch (error) {
        tracing.appendLine(`MCP Server error: ${error instanceof Error ? error.message : String(error)}`);
        
        if (error instanceof McpError) {
          throw error;
        }
        
        throw new McpError(
          ErrorCode.InternalError,
          `Failed to take screenshot: ${error instanceof Error ? error.message : String(error)}`
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
