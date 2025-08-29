// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// OpenSCAD conversion worker process

import * as fs from 'fs';
import * as path from 'path';
import { createOpenSCAD } from 'openscad-wasm-prebuilt';

interface ConversionRequest {
  scadFilePath: string;
  libraryFiles: { [virtualPath: string]: string }; // Base64 encoded content
  workspaceRoot?: string;
  previewMode?: boolean; // Fast preview with lower quality
  timeout?: number; // Custom timeout in milliseconds
}

interface ConversionResponse {
  success: boolean;
  stlPath?: string;
  error?: string;
  progress?: string;
}

// Listen for messages from parent process
process.on('message', async (message: ConversionRequest) => {
  try {
    await convertOpenSCADToSTL(message);
  } catch (error) {
    const response: ConversionResponse = {
      success: false,
      error: error instanceof Error ? error.message : String(error)
    };
    process.send?.(response);
    process.exit(1);
  }
});

async function convertOpenSCADToSTL(request: ConversionRequest): Promise<void> {
  const { scadFilePath, libraryFiles, workspaceRoot, previewMode = false, timeout = 300000 } = request;
  
  try {
    // Send progress update
    sendProgress(`Starting OpenSCAD conversion for: ${scadFilePath}`);
    
    const openscad = await createOpenSCAD({
      print: (text: string) => sendProgress(`OpenSCAD: ${text}`),
      printErr: (text: string) => sendProgress(`OpenSCAD Error: ${text}`),
    });

    // Get direct access to the WASM module
    const instance = openscad.getInstance();

    sendProgress('Loading OpenSCAD libraries...');
    
    // Load library files into virtual filesystem
    for (const [virtualPath, base64Content] of Object.entries(libraryFiles)) {
      try {
        // Ensure directory exists
        const dirPath = virtualPath.substring(0, virtualPath.lastIndexOf('/'));
        if (dirPath) {
          try {
            instance.FS.mkdir(dirPath);
          } catch {
            // Directory might already exist
          }
        }
        
        // Decode base64 content and write to virtual filesystem
        const content = Buffer.from(base64Content, 'base64');
        instance.FS.writeFile(virtualPath, content);
      } catch (error) {
        sendProgress(`Failed to load library file ${virtualPath}: ${error}`);
      }
    }

    const basename = path.basename(scadFilePath, '.scad');
    const dir = path.dirname(scadFilePath);
    const stlPath = path.join(dir, `${basename}.stl`);
    
    // Read the SCAD file content and optionally optimize for preview
    let scadContent = await fs.promises.readFile(scadFilePath, 'utf8');
    
    sendProgress(`Converting OpenSCAD to STL: ${stlPath}`);

    // Write the main SCAD file to the virtual filesystem
    instance.FS.writeFile('/input.scad', scadContent);
    
    // Run OpenSCAD conversion with supported optimizations
    let args = [
      '-o', '/output.stl',
      '/input.scad',
    ];
    
    if (previewMode) {
      // Preview mode: faster but lower quality
      args = args.concat([
        '--preview',                // Use preview mode (faster)
        '--backend=Manifold',    // Use Manifold backend for speed
        '--export-format=binstl',   // Binary STL for smaller size
      ]);
      sendProgress('Using preview mode for faster rendering...');
    } else {
      // Production mode: higher quality but slower
      args = args.concat([
        '--render',                 // Force render mode
        '--export-format=binstl',   // Binary STL for smaller size
      ]);
      sendProgress('Using production mode for high-quality rendering...');
    }
    
    sendProgress(`Running OpenSCAD with args: ${args.join(' ')}`);
    
    // Add timeout handling for long-running operations
    const conversionTimeout = setTimeout(() => {
      sendProgress(`OpenSCAD conversion timeout after ${timeout}ms - operation taking too long`);
      process.exit(1);
    }, timeout);
    
    try {
      instance.callMain(args);
      clearTimeout(conversionTimeout);
    } catch (error) {
      clearTimeout(conversionTimeout);
      throw error;
    }
    
    // Read the output STL from the virtual filesystem
    const stlContent = instance.FS.readFile('/output.stl', { encoding: 'binary' });

    // Write the STL file to the filesystem
    await fs.promises.writeFile(stlPath, stlContent, 'binary');
    
    sendProgress(`OpenSCAD conversion completed successfully: ${stlPath}`);
    
    // Send success response
    const response: ConversionResponse = {
      success: true,
      stlPath
    };
    process.send?.(response);
    process.exit(0);
    
  } catch (error) {
    const response: ConversionResponse = {
      success: false,
      error: error instanceof Error ? error.message : String(error)
    };
    process.send?.(response);
    process.exit(1);
  }
}

function sendProgress(message: string): void {
  const response: ConversionResponse = {
    success: true,
    progress: message
  };
  process.send?.(response);
}

// Handle graceful shutdown
process.on('SIGTERM', () => {
  sendProgress('OpenSCAD conversion process terminated');
  process.exit(0);
});

process.on('SIGINT', () => {
  sendProgress('OpenSCAD conversion process interrupted');
  process.exit(0);
});

// Prevent the process from hanging
process.on('disconnect', () => {
  process.exit(0);
});