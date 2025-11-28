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
  const { scadFilePath, libraryFiles, workspaceRoot, timeout = 300000 } = request;
  
  try {
    // Send progress update
    sendProgress(`Starting OpenSCAD conversion for: ${scadFilePath}`);
    
    const openscad = await createOpenSCAD({
      print: (text: string) => sendProgress(`OpenSCAD: ${text}`),
      printErr: (text: any) => {
        let errorText = text;
        // Check if the error is an object that was toString'd incorrectly
        if (typeof text === 'string' && text.includes('[object Object]')) {
          // Try to access it as an error object
          try {
            const errorObj = text as any;
            if (errorObj && typeof errorObj === 'object') {
              errorText = errorObj.message || errorObj.toString() || text;
            }
          } catch {
            errorText = text;
          }
        } else if (typeof text === 'object' && text !== null) {
          // If it's actually an object, extract the message
          errorText = (text as any).message || JSON.stringify(text);
        }
        sendProgress(`OpenSCAD Error: ${errorText}`);
      },
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
      } catch (error:any) {
        const errorMsg = error instanceof Error ? error.message : String(error);
        sendProgress(`Failed to load library file ${virtualPath}: ${errorMsg}`);
      }
    }

    const basename = path.basename(scadFilePath, '.scad');
    const dir = path.dirname(scadFilePath);
    const stlPath = path.join(dir, `${basename}.stl`);
    
    // Read the SCAD file content
    const scadContent = await fs.promises.readFile(scadFilePath, 'utf8');
    
    sendProgress(`Converting OpenSCAD to STL: ${stlPath}`);

    // Write the main SCAD file to the virtual filesystem
    instance.FS.writeFile('/input.scad', scadContent);
    
    // Run OpenSCAD conversion with supported optimizations
    let args = [
      '-o', '/output.stl',
      '/input.scad',
        '--preview',                // Use preview mode (faster)
        '--backend=Manifold', // Use Manifold backend for speed
        '--export-format=binstl',   // Binary STL for smaller size
      ];
      sendProgress('Using preview mode for faster rendering...');
    
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
    
    const stat: any = instance.FS.stat('/output.stl');
    if (stat) {
      sendProgress(`Output STL file created: ${stat.size || 'unknown'} bytes`);
      
      // Read and verify the output STL from the virtual filesystem
      const stlContent = instance.FS.readFile('/output.stl', { encoding: 'binary' });
      if (stlContent && stlContent.length !== 0) {
        // Write the STL file to the filesystem
        await fs.promises.writeFile(stlPath, stlContent, 'binary');
        
        sendProgress(`OpenSCAD conversion completed successfully: ${stlPath}`);
        
        // Send success response with the STL path
        const response: ConversionResponse = {
          success: true,
          stlPath: stlPath
        };
        process.send?.(response);
        process.exit(0);
      }
    }
    const response: ConversionResponse = {
      success: false,
      error: 'Output file was not created or is empty'
    };
    process.send?.(response);
    process.exit(1);
  } catch (error:any) {
    let errorMessage = 'Unknown error occurred';
    
    if (error instanceof Error) {
      errorMessage = error.message;
    } else if (error && typeof error === 'object') {
      // Handle ErrnoError and other object-based errors
      if (error.name === 'ErrnoError') {
        errorMessage = `File system error (errno ${error.errno || 'unknown'})`;
        if (error.errno === 44) {
          errorMessage = 'OpenSCAD did not generate valid output - check for syntax errors or rendering issues in the SCAD file';
        }
      } else if (error.message) {
        errorMessage = error.message;
      } else {
        try {
          errorMessage = JSON.stringify(error);
        } catch {
          errorMessage = error.toString?.() || 'Error object could not be serialized';
        }
      }
    } else if (error) {
      errorMessage = String(error);
    }
    
    const response: ConversionResponse = {
      success: false,
      error: errorMessage
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