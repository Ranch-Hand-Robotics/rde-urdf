// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// OpenSCAD conversion worker process

import * as fs from 'fs';
import * as path from 'path';
import { createOpenSCAD } from 'openscad-wasm-prebuilt';

type OpenSCADCustomizerValue = string | number | boolean | number[];

interface OpenSCADParameterConfiguration {
  jsonContent: string;
  parameterSetName: string;
}

interface ConversionRequest {
  scadFilePath: string;
  libraryFiles: { [virtualPath: string]: string }; // Base64 encoded content
  workspaceRoot?: string;
  timeout?: number; // Custom timeout in milliseconds
  exportFormat?: 'stl' | 'svg'; // Export format, defaults to 'stl'
  parameterOverrides?: Record<string, OpenSCADCustomizerValue>;
  parameterConfiguration?: OpenSCADParameterConfiguration;
}

interface ConversionResponse {
  success: boolean;
  stlPath?: string;
  svgPath?: string;
  outputPath?: string; // Generic output path for any format
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
    process.send?.(response, undefined, undefined, () => process.exit(1));
  }
});

async function convertOpenSCADToSTL(request: ConversionRequest): Promise<void> {
  const {
    scadFilePath,
    libraryFiles,
    workspaceRoot,
    timeout = 300000,
    exportFormat = 'stl',
    parameterOverrides,
    parameterConfiguration
  } = request;
  
  try {
    // Send progress update
    sendProgress(`Starting OpenSCAD conversion for: ${scadFilePath}`);
    
    const openscad = await createOpenSCAD({
      print: (text: string) => sendProgress(`${text}`),
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

        const normalizedErrorText = String(errorText ?? '').trim();
        // OpenSCAD WASM in worker/virtual FS often logs this benign startup warning.
        if (!normalizedErrorText.includes('Could not initialize localization')) {
          sendProgress(`${errorText}`);
        }
      },
    });

    // Get direct access to the WASM module
    const instance = openscad.getInstance();

    sendProgress('Loading OpenSCAD libraries...');
    
    // Load library files into virtual filesystem
    for (const [virtualPath, base64Content] of Object.entries(libraryFiles)) {
      try {
        // Ensure all parent directories exist (recursive mkdir)
        const dirPath = virtualPath.substring(0, virtualPath.lastIndexOf('/'));
        if (dirPath) {
          const segments = dirPath.split('/').filter(s => s.length > 0);
          let current = '';
          for (const segment of segments) {
            current += '/' + segment;
            try { instance.FS.mkdir(current); } catch { /* already exists */ }
          }
        }
        
        // Decode base64 content and write to virtual filesystem
        const content = Buffer.from(base64Content, 'base64');
        instance.FS.writeFile(virtualPath, content);
      } catch (error: any) {
        const errorMsg = error instanceof Error ? error.message : String(error);
        sendProgress(`Failed to load library file ${virtualPath}: ${errorMsg}`);
      }
    }

    const basename = path.basename(scadFilePath, '.scad');
    const dir = path.dirname(scadFilePath);
    
    // Determine output format and paths
    const fileExtension = exportFormat === 'svg' ? '.svg' : '.stl';
    const outputPath = path.join(dir, `${basename}${fileExtension}`);
    const virtualOutputPath = exportFormat === 'svg' ? '/output.svg' : '/output.stl';
    
    // Read the SCAD file content
    const scadContent = await fs.promises.readFile(scadFilePath, 'utf8');
    
    sendProgress(`Converting OpenSCAD to ${exportFormat.toUpperCase()}: ${outputPath}`);

    // Write the main SCAD file to the virtual filesystem
    instance.FS.writeFile('/input.scad', scadContent);

    // Optional: Write parameter configuration file for -p/-P support
    if (parameterConfiguration?.jsonContent && parameterConfiguration?.parameterSetName) {
      instance.FS.writeFile('/input.parameters.json', parameterConfiguration.jsonContent);
    }
    
    // Run OpenSCAD conversion with appropriate settings for format
    let args: string[] = [
      '-o', virtualOutputPath
    ];

    // Append customizer variable overrides using native OpenSCAD CLI behavior.
    const overrideArgs = buildOverrideArgs(parameterOverrides);
    if (overrideArgs.length > 0) {
      args.push(...overrideArgs);
    }

    // Append parameter configuration set arguments if supplied.
    if (parameterConfiguration?.jsonContent && parameterConfiguration?.parameterSetName) {
      args.push('-p', '/input.parameters.json', '-P', parameterConfiguration.parameterSetName);
    }

    if (exportFormat === 'svg') {
      sendProgress('Exporting to SVG format...');
    } else {
      // STL export for 3D designs with optimizations
      args.push(
        '--preview',                // Use preview mode (faster)
        '--backend=Manifold',       // Use Manifold backend for speed
        '--export-format=binstl'    // Binary STL for smaller size
      );
      sendProgress('Using preview mode for faster rendering...');
    }

    // Input file should be last argument.
    args.push('/input.scad');
    
    sendProgress(`Running OpenSCAD with args: ${args.join(' ')}`);
    
    // Add timeout handling for long-running operations
    const conversionTimeout = setTimeout(() => {
      sendProgress(`OpenSCAD conversion timeout after ${timeout}ms - operation taking too long`, () => process.exit(1));
    }, timeout);
    
    try {
      instance.callMain(args);
      clearTimeout(conversionTimeout);
    } catch (error) {
      clearTimeout(conversionTimeout);
      throw error;
    }
    
    const stat: any = instance.FS.stat(virtualOutputPath);
    if (stat) {
      sendProgress(`Output ${exportFormat.toUpperCase()} file created: ${stat.size || 'unknown'} bytes`);
      
      // Read and verify the output from the virtual filesystem
      const encoding = exportFormat === 'svg' ? 'utf8' : 'binary';
      const outputContent = instance.FS.readFile(virtualOutputPath, { encoding: encoding as any });
      if (outputContent && outputContent.length !== 0) {
        // Write the output file to the filesystem
        await fs.promises.writeFile(outputPath, outputContent, encoding as any);
        
        // Send success response with the output path
        const response: ConversionResponse = {
          success: true,
          outputPath: outputPath,
          // Keep backwards compatibility
          ...(exportFormat === 'stl' && { stlPath: outputPath }),
          ...(exportFormat === 'svg' && { svgPath: outputPath }),
        };
        process.send?.(response, undefined, undefined, () => process.exit(0));
        return;
      }
    }
    const response: ConversionResponse = {
      success: false,
      error: 'Output file was not created or is empty'
    };
    process.send?.(response, undefined, undefined, () => {
      process.exit(1);
    });
  } catch (error: any) {
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
    process.send?.(response, undefined, undefined, () => {
      process.exit(1);
    });
  }
}

function serializeValue(value: OpenSCADCustomizerValue): string {
  if (typeof value === 'string') {
    return JSON.stringify(value);
  }
  if (typeof value === 'boolean') {
    return value ? 'true' : 'false';
  }
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) {
      throw new Error('Invalid numeric parameter override value (must be finite).');
    }
    return `${value}`;
  }
  if (Array.isArray(value)) {
    if (value.length > 4) {
      throw new Error('Vector parameter override length must be <= 4.');
    }
    for (const item of value) {
      if (typeof item !== 'number' || !Number.isFinite(item)) {
        throw new Error('Vector parameter overrides must contain finite numeric values only.');
      }
    }
    return `[${value.join(', ')}]`;
  }

  throw new Error('Unsupported parameter override value type.');
}

function buildOverrideArgs(parameterOverrides?: Record<string, OpenSCADCustomizerValue>): string[] {
  if (!parameterOverrides) {
    return [];
  }

  const identifierRegex = /^[A-Za-z_][A-Za-z0-9_]*$/;
  const args: string[] = [];

  for (const [name, value] of Object.entries(parameterOverrides)) {
    if (!identifierRegex.test(name)) {
      continue;
    }
    args.push('-D', `${name}=${serializeValue(value)}`);
  }

  return args;
}

function sendProgress(message: string, callback?: () => void): void {
  const response: ConversionResponse = {
    success: true,
    progress: message
  };
  process.send?.(response, undefined, undefined, callback);
}

// Handle graceful shutdown
process.on('SIGTERM', () => {
  sendProgress('OpenSCAD conversion process terminated', () => process.exit(0));
});

process.on('SIGINT', () => {
  sendProgress('OpenSCAD conversion process interrupted', () => process.exit(0));
});

// Prevent the process from hanging
process.on('disconnect', () => {
  process.exit(0);
});