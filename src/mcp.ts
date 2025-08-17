import { StreamableHTTPServerTransport } from '@modelcontextprotocol/sdk/server/streamableHttp.js';
import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import { 
  ErrorCode,
  McpError,
  isInitializeRequest,
} from '@modelcontextprotocol/sdk/types.js';
import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';
import * as BABYLON from 'babylonjs';
import * as urdf from '@polyhobbyist/babylon_ros';
import * as utils from './utils';
import { tracing } from './extension';
import { createOpenSCAD } from 'openscad-wasm-prebuilt';
import * as express from 'express';
import { randomUUID } from 'node:crypto';

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
        name: 'urdf-screenshot-server',
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
    this.server.tool('take_urdf_screenshot', {
      description: 'When you update a URDF, Xacro, or OpenSCAD file, use this to capture an image and verify your work.',
      inputSchema: {
        type: 'object',
        properties: {
          width: {
            type: 'number',
            description: 'Screenshot width in pixels (optional, default: 800)',
            default: 800,
          },
          height: {
            type: 'number',
            description: 'Screenshot height in pixels (optional, default: 600)',
            default: 600,
          },
        },
        required: [],
      },
    }, async (args) => {
      try {
        const { width = 800, height = 600 } = args as {
          width?: number;
          height?: number;
        };

        // Get the currently active editor from VS Code
        const activeEditor = vscode.window.activeTextEditor;
        if (!activeEditor) {
          throw new McpError(
            ErrorCode.InvalidParams,
            'No active editor found. Please open a URDF, Xacro, or OpenSCAD file in VS Code.'
          );
        }

        const document = activeEditor.document;
        const filePath = document.uri.fsPath;
        
        tracing.appendLine(`MCP Server: Taking screenshot of active file ${filePath} (${width}x${height})`);

        // Validate file extension
        const ext = path.extname(filePath).toLowerCase();
        if (!['.urdf', '.xacro', '.scad'].includes(ext)) {
          throw new McpError(
            ErrorCode.InvalidParams,
            `Unsupported file type: ${ext}. Please open a URDF (.urdf), Xacro (.xacro), or OpenSCAD (.scad) file. Current file: ${path.basename(filePath)}`
          );
        }

        // Check if the document has unsaved changes
        if (document.isDirty) {
          tracing.appendLine('Document has unsaved changes, using editor content instead of file content');
        }

        // Take screenshot using the current file
        const base64Image = await this.takeScreenshot(filePath, width, height, document);

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
   * Takes a screenshot of a URDF, Xacro, or OpenSCAD file
   */
  private async takeScreenshot(filePath: string, width: number, height: number, document?: vscode.TextDocument): Promise<string> {
    return new Promise(async (resolve, reject) => {
      try {
        // Create a headless engine for rendering
        const engine = new BABYLON.NullEngine({
          renderWidth: width,
          renderHeight: height,
          textureSize: 512,
          deterministicLockstep: false,
          lockstepMaxSteps: 1,
        });

        const scene = new BABYLON.Scene(engine);
        
        // Configure scene background
        const config = vscode.workspace.getConfiguration('urdf-editor');
        const backgroundColor = config.get<string>('BackgroundColor', '#000000');
        scene.clearColor = BABYLON.Color4.FromHexString(backgroundColor + 'FF');

        // Create camera
        const camera = new BABYLON.ArcRotateCamera('camera', 0, 0, 10, BABYLON.Vector3.Zero(), scene);
        camera.setTarget(BABYLON.Vector3.Zero());

        // Create lighting
        const light = new BABYLON.HemisphericLight('light', new BABYLON.Vector3(0, 1, 0), scene);
        light.intensity = 0.7;
        
        // Add directional light for better visibility
        const dirLight = new BABYLON.DirectionalLight('dirLight', new BABYLON.Vector3(-1, -1, -1), scene);
        dirLight.intensity = 0.5;

        // Create RobotScene instance
        const robotScene = new urdf.RobotScene();
        robotScene.scene = scene;
        robotScene.engine = engine;
        robotScene.camera = camera;

        // Process file based on extension
        const ext = path.extname(filePath).toLowerCase();
        
        if (ext === '.scad') {
          await this.loadOpenSCADFile(robotScene, filePath, document);
        } else {
          // Handle URDF/Xacro files
          await this.loadUrdfFile(robotScene, filePath, document);
        }

        // Wait for scene to be ready
        await scene.whenReadyAsync();

        // Position camera to view the model
        this.positionCameraForModel(robotScene);

        // Render multiple frames to ensure everything is loaded
        for (let i = 0; i < 3; i++) {
          scene.render();
        }

        // Take screenshot using the RobotScene method
        const base64Image = await robotScene.takeScreenshot(width, height);
        
        // Cleanup
        scene.dispose();
        engine.dispose();

        resolve(base64Image);
      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Loads an OpenSCAD file by converting it to STL first
   */
  private async loadOpenSCADFile(robotScene: urdf.RobotScene, filePath: string, document?: vscode.TextDocument): Promise<void> {
    try {
      // Read OpenSCAD content from document or file
      let scadContent: string;
      if (document && document.uri.fsPath === filePath) {
        // Use current document content (including unsaved changes)
        scadContent = document.getText();
        tracing.appendLine('Using OpenSCAD content from active editor (including unsaved changes)');
      } else {
        // Fall back to reading from file
        scadContent = fs.readFileSync(filePath, 'utf8');
      }
      
      tracing.appendLine(`Converting OpenSCAD file to STL: ${filePath}`);
      
      // Create OpenSCAD instance
      const openscad = await createOpenSCAD({
        // Optional callbacks for stdout/stderr
        print: (text: string) => tracing.appendLine(`OpenSCAD: ${text}`),
        printErr: (text: string) => tracing.appendLine(`OpenSCAD Error: ${text}`),
      });

      // Convert to STL
      const stlData = await openscad.renderToStl(scadContent);
      
      // Since this is dropping an stl in the same directory, go with it.
      const tempStlPath = filePath.replace('.scad', '.stl');
      fs.writeFileSync(tempStlPath, stlData);

      // Load the STL as a 3D mesh
      if (robotScene.scene) {
        const scale = BABYLON.Vector3.One();
        const mesh = new urdf.Mesh(`file://${tempStlPath}`, scale);
        
        const visual = new urdf.Visual();
        visual.material = new urdf.Material();
        visual.material.name = 'default';
        visual.geometry = mesh;

        const link = new urdf.Link();
        link.visuals.push(visual);

        const robot = new urdf.Robot();
        robot.links.set('base_link', link);
        
        robotScene.currentRobot = robot;
        await robot.create(robotScene.scene);
      }

      // Clean up temporary file
      if (fs.existsSync(tempStlPath)) {
        fs.unlinkSync(tempStlPath);
      }
      
      tracing.appendLine('OpenSCAD conversion completed successfully');
    } catch (error) {
      throw new Error(`Failed to load OpenSCAD file: ${error instanceof Error ? error.message : String(error)}`);
    }
  }

  /**
   * Loads a URDF or Xacro file
   */
  private async loadUrdfFile(robotScene: urdf.RobotScene, filePath: string, document?: vscode.TextDocument): Promise<void> {
    try {
      let urdfContent: string;
      
      const ext = path.extname(filePath).toLowerCase();
      if (ext === '.xacro') {
        // Process Xacro file from disk
        // Note: Xacro processing requires file system access for includes and package resolution
        // If you have unsaved changes, save the file first for them to be included
        if (document && document.uri.fsPath === filePath && document.isDirty) {
          tracing.appendLine('Note: Document has unsaved changes. Save the file to include latest changes in Xacro processing.');
        }
        const [processedUrdf] = await utils.processXacro(filePath, (packageUri: vscode.Uri) => {
          return packageUri.fsPath;
        });
        urdfContent = processedUrdf;
      } else {
        // Handle URDF files
        if (document && document.uri.fsPath === filePath) {
          // Use current document content (including unsaved changes)
          urdfContent = document.getText();
          tracing.appendLine('Using URDF content from active editor (including unsaved changes)');
        } else {
          // Read URDF file directly from disk
          urdfContent = fs.readFileSync(filePath, 'utf8');
        }
        urdfContent = utils.processUrdfContent(urdfContent);
      }

      // Apply URDF to scene
      if (robotScene.scene) {
        await robotScene.applyURDF(urdfContent);
      }
    } catch (error) {
      throw new Error(`Failed to load URDF/Xacro file: ${error instanceof Error ? error.message : String(error)}`);
    }
  }

  /**
   * Positions the camera to properly view the loaded model
   */
  private positionCameraForModel(robotScene: urdf.RobotScene): void {
    if (!robotScene.camera || !robotScene.scene) {
      return;
    }

    // Get bounding box of all meshes in the scene
    const meshes = robotScene.scene.meshes.filter((mesh: any) => mesh.isVisible);
    
    if (meshes.length > 0) {
      // Calculate bounding box
      let min = new BABYLON.Vector3(Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE);
      let max = new BABYLON.Vector3(Number.MIN_VALUE, Number.MIN_VALUE, Number.MIN_VALUE);

      meshes.forEach((mesh: any) => {
        const boundingInfo = mesh.getBoundingInfo();
        const meshMin = boundingInfo.minimum;
        const meshMax = boundingInfo.maximum;

        min = BABYLON.Vector3.Minimize(min, meshMin);
        max = BABYLON.Vector3.Maximize(max, meshMax);
      });

      // Calculate center and size
      const center = BABYLON.Vector3.Center(min, max);
      const size = max.subtract(min);
      const distance = Math.max(size.x, size.y, size.z) * 2;

      // Position camera
      robotScene.camera.setTarget(center);
      robotScene.camera.radius = distance;
      robotScene.camera.alpha = Math.PI / 4; // 45 degrees
      robotScene.camera.beta = Math.PI / 3;  // 60 degrees
    } else {
      // Default camera position if no meshes found
      robotScene.camera.setTarget(BABYLON.Vector3.Zero());
      robotScene.camera.radius = 5;
      robotScene.camera.alpha = Math.PI / 4;
      robotScene.camera.beta = Math.PI / 3;
    }
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
        vscode.window.showInformationMessage(`URDF MCP Server started on port ${this.port}`);
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
      vscode.window.showInformationMessage('URDF MCP Server stopped');
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
