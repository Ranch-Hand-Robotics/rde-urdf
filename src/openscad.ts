// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Additional features added by PolyHobbyist

import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import * as os from 'os';
import { createOpenSCAD } from 'openscad-wasm-prebuilt';

// Type declarations for openscad-wasm
export interface OpenSCADInstance {
  FS: {
    writeFile(path: string, data: string | Uint8Array): void;
    readFile(path: string, options: { encoding: string }): string;
    mkdir(path: string): void;
  };
  callMain(args: string[]): void;
}

export interface OpenSCAD {
  getInstance(): OpenSCADInstance;
}

export interface OpenSCADModule {
  createOpenSCAD(options?: {
    print?: (text: string) => void;
    printErr?: (text: string) => void;
  }): Promise<OpenSCAD>;
}

/**
 * Get default OpenSCAD library paths based on the operating system
 */
export function getDefaultOpenSCADLibraryPaths(): string[] {
  const platform = os.platform();
  const homeDir = os.homedir();
  const paths: string[] = [];

  switch (platform) {
    case 'win32':
      // Windows: My Documents\OpenSCAD\libraries
      const documentsPath = path.join(homeDir, 'Documents', 'OpenSCAD', 'libraries');
      paths.push(documentsPath);
      break;
    case 'linux':
      // Linux: $HOME/.local/share/OpenSCAD/libraries
      const linuxLibPath = path.join(homeDir, '.local', 'share', 'OpenSCAD', 'libraries');
      paths.push(linuxLibPath);
      break;
    case 'darwin':
      // macOS: $HOME/Documents/OpenSCAD/libraries
      const macLibPath = path.join(homeDir, 'Documents', 'OpenSCAD', 'libraries');
      paths.push(macLibPath);
      break;
  }

  return paths;
}

/**
 * Resolve workspace variables in library paths
 */
export function resolveWorkspaceVariables(libraryPath: string, workspaceRoot?: string): string {
  if (workspaceRoot && libraryPath.includes('${workspace}')) {
    return libraryPath.replace(/\$\{workspace\}/g, workspaceRoot);
  }
  return libraryPath;
}

/**
 * Get all OpenSCAD library paths (default + user-configured), filtered to existing directories
 */
export async function getAllOpenSCADLibraryPaths(workspaceRoot?: string): Promise<string[]> {
  const config = vscode.workspace.getConfiguration('urdf-editor');
  const userLibraryPaths: string[] = config.get('OpenSCADLibraryPaths', []);
  
  // Get default OS-specific paths
  const defaultPaths = getDefaultOpenSCADLibraryPaths();
  
  // Resolve workspace variables in user paths
  const resolvedUserPaths = userLibraryPaths.map(p => resolveWorkspaceVariables(p, workspaceRoot));
  
  // Combine all paths
  const allPaths = [...defaultPaths, ...resolvedUserPaths];
  
  // Filter to only existing directories
  const existingPaths: string[] = [];
  for (const libPath of allPaths) {
    try {
      const stat = await vscode.workspace.fs.stat(vscode.Uri.file(libPath));
      if (stat.type === vscode.FileType.Directory) {
        existingPaths.push(libPath);
      }
    } catch {
      // Directory doesn't exist, skip it
    }
  }
  
  return existingPaths;
}

/**
 * Load all library files from the specified library paths into the OpenSCAD virtual filesystem
 */
export async function loadLibraryFiles(instance: OpenSCADInstance, libraryPaths: string[], trace: vscode.OutputChannel): Promise<void> {
  for (const libPath of libraryPaths) {
    try {
      trace.appendLine(`Loading OpenSCAD library from: ${libPath}`);
      // Mirror the actual filesystem structure in the virtual filesystem
      const libraryName = path.basename(libPath);
      const virtualLibraryRoot = `/`;  // put them all in the same place
      await loadLibraryDirectory(instance, libPath, '', virtualLibraryRoot, trace);
    } catch (error) {
      trace.appendLine(`Failed to load library from ${libPath}: ${error}`);
    }
  }
}

/**
 * Recursively load files from a library directory into the OpenSCAD virtual filesystem
 */
export async function loadLibraryDirectory(
  instance: OpenSCADInstance, 
  basePath: string, 
  relativePath: string, 
  virtualRoot: string,
  trace: vscode.OutputChannel
): Promise<void> {
  const fullPath = `${basePath}/${relativePath}`;
  
  try {
    // Use vscode.workspace.findFiles to find all files in the directory
    const pattern = new vscode.RelativePattern(fullPath, '**/*');
    const files = await vscode.workspace.findFiles(pattern);
    
    for (const fileUri of files) {
      // Get relative path from base path
      const filePath = fileUri.path;
      const relativeFilePath = filePath.substring(fullPath.length + 1);
      const virtualPath = `${virtualRoot}/${relativeFilePath}`;
      
      try {
        // Create directory in virtual filesystem
        const dirPath = virtualPath.substring(0, virtualPath.lastIndexOf('/'));
        if (dirPath) {
          try {
            instance.FS.mkdir(dirPath);
          } catch {
            // ignore if directory already exists
          }
        }

        // Read and write file
        const content = await vscode.workspace.fs.readFile(fileUri);
        instance.FS.writeFile(virtualPath, content);
      } catch {
        trace.appendLine(`Failed to load file ${relativeFilePath}`);
      }
    }
  } catch (error) {
    trace.appendLine(`Failed to read directory ${fullPath}: ${error}`);
  }
}

/**
 * Check if a file is an OpenSCAD file based on its extension
 */
export function isOpenSCADFile(filePath: string): boolean {
  return path.extname(filePath).toLowerCase() === '.scad';
}

/**
 * Convert an OpenSCAD file to STL format using openscad-wasm (original implementation)
 */
export async function convertOpenSCADToSTL(scadFilePath: string, trace: vscode.OutputChannel): Promise<string | null> {
  try {
    trace.appendLine(`Starting OpenSCAD conversion for: ${scadFilePath}`);
    
    const openscad = await createOpenSCAD({
      // Optional callbacks for stdout/stderr
      print: (text: string) => trace.appendLine(`OpenSCAD: ${text}`),
      printErr: (text: string) => trace.appendLine(`OpenSCAD Error: ${text}`),
    });

    // Get direct access to the WASM module
    const instance = openscad.getInstance();

    // Get workspace root for variable resolution
    const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.path;
    
    // Load OpenSCAD libraries
    trace.appendLine(`Loading OpenSCAD libraries...`);
    const libraryPaths = await getAllOpenSCADLibraryPaths(workspaceRoot);
    if (libraryPaths.length > 0) {
      trace.appendLine(`Found ${libraryPaths.length} library paths: ${libraryPaths.join(', ')}`);
      await loadLibraryFiles(instance, libraryPaths, trace);
    } else {
      trace.appendLine(`No OpenSCAD library paths found`);
    }

    const basename = scadFilePath.substring(scadFilePath.lastIndexOf('/') + 1, scadFilePath.lastIndexOf('.scad'));
    const dir = scadFilePath.substring(0, scadFilePath.lastIndexOf('/'));
    const stlPath = `${dir}/${basename}.stl`;
    
    trace.appendLine(`Converting OpenSCAD to STL: ${stlPath}`);

    const scadUri = vscode.Uri.file(scadFilePath);
    const scadText = new TextDecoder().decode(await vscode.workspace.fs.readFile(scadUri));
    
    // Write the main SCAD file to the virtual filesystem
    instance.FS.writeFile('/input.scad', scadText);
    
    // Add library search paths to OpenSCAD arguments
    const args = ['-o', '/output.stl', '/input.scad'];

    // Run OpenSCAD with library paths
    instance.callMain(args);
    
    // Read the output STL from the virtual filesystem
    const stlContent = instance.FS.readFile('/output.stl', { encoding: 'binary' });

    // Use the filesystem API directly
    await vscode.workspace.fs.writeFile(vscode.Uri.file(stlPath), new Uint8Array(stlContent));
    
    trace.appendLine(`OpenSCAD conversion completed successfully: ${stlPath}`);
    return stlPath;
  } catch (error) {
    trace.appendLine(`OpenSCAD conversion failed`);
    vscode.window.showErrorMessage(`Failed to convert OpenSCAD file, check output for more info`);
    return null;
  }
}

/**
 * Convert an OpenSCAD file to STL format using a killable child process
 */
export async function convertOpenSCADToSTLCancellable(
  scadFilePath: string, 
  trace: vscode.OutputChannel, 
  token?: vscode.CancellationToken,
  options?: {
    previewMode?: boolean;  // Use faster preview mode
    timeout?: number;       // Custom timeout in milliseconds (default: 5 minutes)
  }
): Promise<string | null> {
  return new Promise(async (resolve, reject) => {
    let childProcess: any = null;
    let cancelled = false;
    
    // Set up cancellation handling
    const cancellationListener = token?.onCancellationRequested(() => {
      cancelled = true;
      trace.appendLine(`OpenSCAD conversion cancelled: ${scadFilePath}`);
      if (childProcess) {
        trace.appendLine('Killing OpenSCAD worker process...');
        childProcess.kill('SIGTERM');
        // Force kill if it doesn't respond to SIGTERM
        setTimeout(() => {
          if (childProcess && !childProcess.killed) {
            childProcess.kill('SIGKILL');
          }
        }, 5000);
      }
      resolve(null);
    });

    try {
      // Check if already cancelled
      if (token?.isCancellationRequested) {
        resolve(null);
        return;
      }

      trace.appendLine(`Starting process-based OpenSCAD conversion for: ${scadFilePath}`);
      
      // Get workspace root for variable resolution
      const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.path;
      
      // Load OpenSCAD libraries and encode them as base64
      trace.appendLine(`Loading OpenSCAD libraries...`);
      const libraryPaths = await getAllOpenSCADLibraryPaths(workspaceRoot);
      const libraryFiles: { [virtualPath: string]: string } = {};
      
      if (cancelled) {
        resolve(null);
        return;
      }

      if (libraryPaths.length > 0) {
        trace.appendLine(`Found ${libraryPaths.length} library paths: ${libraryPaths.join(', ')}`);
        await loadLibraryFilesBase64(libraryPaths, libraryFiles, trace);
      } else {
        trace.appendLine(`No OpenSCAD library paths found`);
      }

      if (cancelled) {
        resolve(null);
        return;
      }

      // Spawn the worker process
      const { spawn } = await import('child_process');
      const workerPath = path.join(__dirname, 'workers', 'openscadWorker.js');
      
      childProcess = spawn(process.execPath, [workerPath], {
        stdio: ['pipe', 'pipe', 'pipe', 'ipc'],
        detached: false
      });

      if (cancelled) {
        if (childProcess) {
          childProcess.kill('SIGTERM');
        }
        resolve(null);
        return;
      }

      // Handle process events
      childProcess.on('error', (error: Error) => {
        if (!cancelled) {
          trace.appendLine(`Worker process error: ${error.message}`);
          reject(error);
        }
      });

      childProcess.on('exit', (code: number, signal: string) => {
        if (!cancelled) {
          if (code === 0) {
            // Success case is handled by the message handler
          } else {
            trace.appendLine(`Worker process exited with code ${code}, signal ${signal}`);
            resolve(null);
          }
        }
      });

      // Handle messages from worker
      childProcess.on('message', (response: any) => {
        if (cancelled) {
          return;
        }

        if (response.progress) {
          trace.appendLine(response.progress);
        } else if (response.success && response.stlPath) {
          trace.appendLine(`OpenSCAD conversion completed successfully: ${response.stlPath}`);
          resolve(response.stlPath);
        } else if (!response.success && response.error) {
          trace.appendLine(`OpenSCAD conversion failed: ${response.error}`);
          vscode.window.showErrorMessage(`Failed to convert OpenSCAD file: ${response.error}`);
          resolve(null);
        }
      });

      // Send conversion request to worker
      const request = {
        scadFilePath,
        libraryFiles,
        workspaceRoot,
        previewMode: options?.previewMode || false,
        timeout: options?.timeout || 300000  // Default 5 minutes
      };

      childProcess.send(request);
      
    } catch (error) {
      if (!cancelled) {
        trace.appendLine(`OpenSCAD conversion failed: ${error}`);
        vscode.window.showErrorMessage(`Failed to convert OpenSCAD file, check output for more info`);
        reject(error);
      } else {
        resolve(null);
      }
    } finally {
      cancellationListener?.dispose();
    }
  });
}

/**
 * Load library files and encode them as base64 for transmission to worker process
 */
async function loadLibraryFilesBase64(
  libraryPaths: string[], 
  libraryFiles: { [virtualPath: string]: string },
  trace: vscode.OutputChannel
): Promise<void> {
  for (const libPath of libraryPaths) {
    try {
      trace.appendLine(`Loading OpenSCAD library from: ${libPath}`);
      await loadLibraryDirectoryBase64(libPath, '', '/', libraryFiles, trace);
    } catch (error) {
      trace.appendLine(`Failed to load library from ${libPath}: ${error}`);
    }
  }
}

/**
 * Recursively load files from a library directory and encode as base64
 */
async function loadLibraryDirectoryBase64(
  basePath: string, 
  relativePath: string, 
  virtualRoot: string,
  libraryFiles: { [virtualPath: string]: string },
  trace: vscode.OutputChannel
): Promise<void> {
  const fullPath = `${basePath}/${relativePath}`;
  
  try {
    // Use vscode.workspace.findFiles to find all files
    const pattern = new vscode.RelativePattern(fullPath, '**/*');
    const files = await vscode.workspace.findFiles(pattern);
    
    for (const fileUri of files) {
      const filePath = fileUri.path;
      const relativeFilePath = filePath.substring(fullPath.length + 1);
      const virtualPath = `${virtualRoot}/${relativeFilePath}`;
      
      try {
        const content = await vscode.workspace.fs.readFile(fileUri);
        // Encode as base64 for transmission to worker process
        libraryFiles[virtualPath] = btoa(String.fromCharCode(...content));
      } catch {
        trace.appendLine(`Failed to load file ${relativeFilePath}`);
      }
    }
  } catch (error) {
    trace.appendLine(`Failed to read directory ${fullPath}: ${error}`);
  }
}

// Types for library documentation
export interface OpenSCADModuleInfo {
  name: string;
  signature: string;
  comment?: string;
  parameters?: string[];
}

export interface OpenSCADLibraryInfo {
  filePath: string;
  relativePath: string;
  headerComment?: string;
  modules: OpenSCADModuleInfo[];
}

export interface OpenSCADLibrariesDocumentation {
  generatedAt: string;
  libraryPaths: string[];
  libraries: OpenSCADLibraryInfo[];
}

/**
 * Extract header comment from OpenSCAD file content
 */
function extractHeaderComment(content: string): string | undefined {
  const lines = content.split('\n');
  const headerLines: string[] = [];
  let inHeaderComment = false;
  
  for (const line of lines) {
    const trimmedLine = line.trim();
    
    // Skip empty lines at the beginning
    if (!inHeaderComment && trimmedLine === '') {
      continue;
    }
    
    // Check for start of block comment
    if (!inHeaderComment && trimmedLine.startsWith('/*')) {
      inHeaderComment = true;
      headerLines.push(trimmedLine.substring(2).trim());
      
      // Check if it's a single-line block comment
      if (trimmedLine.endsWith('*/')) {
        headerLines[headerLines.length - 1] = headerLines[headerLines.length - 1].slice(0, -2).trim();
        break;
      }
      continue;
    }
    
    // Check for end of block comment
    if (inHeaderComment && trimmedLine.endsWith('*/')) {
      headerLines.push(trimmedLine.slice(0, -2).trim());
      break;
    }
    
    // Check for line comment
    if (!inHeaderComment && trimmedLine.startsWith('//')) {
      headerLines.push(trimmedLine.substring(2).trim());
      continue;
    }
    
    // Continue collecting block comment lines
    if (inHeaderComment) {
      // Remove leading * if present
      const commentLine = trimmedLine.startsWith('*') ? trimmedLine.substring(1).trim() : trimmedLine;
      headerLines.push(commentLine);
      continue;
    }
    
    // If we hit non-comment content and haven't found a header comment, stop
    if (trimmedLine !== '' && !trimmedLine.startsWith('//') && !trimmedLine.startsWith('/*')) {
      break;
    }
  }
  
  return headerLines.length > 0 ? headerLines.join('\n').trim() : undefined;
}

/**
 * Extract module definitions and their comments from OpenSCAD content
 */
function extractModules(content: string): OpenSCADModuleInfo[] {
  const modules: OpenSCADModuleInfo[] = [];
  const lines = content.split('\n');
  
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i].trim();
    
    // Look for module definitions
    const moduleMatch = line.match(/^module\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*\((.*?)\)/);
    if (moduleMatch) {
      const moduleName = moduleMatch[1];
      const signature = line;
      
      // Extract parameters
      const paramString = moduleMatch[2];
      const parameters = paramString
        .split(',')
        .map(p => p.trim())
        .filter(p => p.length > 0);
      
      // Look for preceding comment
      let comment: string | undefined;
      let commentLines: string[] = [];
      
      // Look backwards for comments
      for (let j = i - 1; j >= 0; j--) {
        const prevLine = lines[j].trim();
        
        if (prevLine === '') {
          continue; // Skip empty lines
        }
        
        if (prevLine.startsWith('//')) {
          commentLines.unshift(prevLine.substring(2).trim());
        } else if (prevLine.startsWith('/*') && prevLine.endsWith('*/')) {
          // Single line block comment
          commentLines.unshift(prevLine.substring(2, prevLine.length - 2).trim());
        } else {
          break; // Stop at non-comment line
        }
      }
      
      if (commentLines.length > 0) {
        comment = commentLines.join('\n');
      }
      
      modules.push({
        name: moduleName,
        signature,
        comment,
        parameters: parameters.length > 0 ? parameters : undefined
      });
    }
    
    // Also look for function definitions
    const functionMatch = line.match(/^function\s+([a-zA-Z_][a-zA-Z0-9_]*)\s*\((.*?)\)/);
    if (functionMatch) {
      const functionName = functionMatch[1];
      const signature = line;
      
      // Extract parameters
      const paramString = functionMatch[2];
      const parameters = paramString
        .split(',')
        .map(p => p.trim())
        .filter(p => p.length > 0);
      
      // Look for preceding comment (same logic as modules)
      let comment: string | undefined;
      let commentLines: string[] = [];
      
      for (let j = i - 1; j >= 0; j--) {
        const prevLine = lines[j].trim();
        
        if (prevLine === '') {
          continue;
        }
        
        if (prevLine.startsWith('//')) {
          commentLines.unshift(prevLine.substring(2).trim());
        } else if (prevLine.startsWith('/*') && prevLine.endsWith('*/')) {
          commentLines.unshift(prevLine.substring(2, prevLine.length - 2).trim());
        } else {
          break;
        }
      }
      
      if (commentLines.length > 0) {
        comment = commentLines.join('\n');
      }
      
      modules.push({
        name: functionName,
        signature,
        comment,
        parameters: parameters.length > 0 ? parameters : undefined
      });
    }
  }
  
  return modules;
}

/**
 * Generate documentation for all OpenSCAD libraries
 */
export async function generateOpenSCADLibrariesDocumentation(workspaceRoot?: string): Promise<OpenSCADLibrariesDocumentation> {
  const libraryPaths = await getAllOpenSCADLibraryPaths(workspaceRoot);
  const libraries: OpenSCADLibraryInfo[] = [];
  
  for (const libPath of libraryPaths) {
    await processLibraryDirectory(libPath, '', libraries);
  }
  
  return {
    generatedAt: new Date().toISOString(),
    libraryPaths,
    libraries
  };
}

/**
 * Recursively process a library directory to extract documentation
 */
async function processLibraryDirectory(basePath: string, relativePath: string, libraries: OpenSCADLibraryInfo[]): Promise<void> {
  const fullPath = `${basePath}/${relativePath}`;
  
  try {
    // Use vscode.workspace.findFiles to find all .scad files
    const pattern = new vscode.RelativePattern(fullPath, '**/*.scad');
    const files = await vscode.workspace.findFiles(pattern);
    
    for (const fileUri of files) {
      const filePath = fileUri.path;
      const entryRelativePath = filePath.substring(fullPath.length + 1);
      
      try {
        const contentBuffer = await vscode.workspace.fs.readFile(fileUri);
        const content = new TextDecoder().decode(contentBuffer);
        const headerComment = extractHeaderComment(content);
        const modules = extractModules(content);
        
        libraries.push({
          filePath,
          relativePath: entryRelativePath,
          headerComment,
          modules
        });
      } catch {
        // ignore since this is doc generation
      }
    }
  } catch  {
    // ignore since this is doc generation
  }
}

/**
 * Convert OpenSCAD libraries documentation to Markdown format
 */
export function convertLibrariesDocumentationToMarkdown(doc: OpenSCADLibrariesDocumentation): string {
  const lines: string[] = [];
  
  lines.push('# OpenSCAD Libraries Documentation');
  lines.push('');
  lines.push(`Generated at: ${doc.generatedAt}`);
  lines.push('');
  
  if (doc.libraryPaths.length > 0) {
    lines.push('## Library Paths');
    lines.push('');
    for (const libPath of doc.libraryPaths) {
      lines.push(`- \`${libPath}\``);
    }
    lines.push('');
  }
  
  if (doc.libraries.length === 0) {
    lines.push('No OpenSCAD libraries found.');
    return lines.join('\n');
  }
  
  lines.push('## Available Libraries');
  lines.push('');
  
  for (const library of doc.libraries) {
    lines.push(`### ${library.relativePath}`);
    lines.push('');
    lines.push(`**File Path:** \`${library.filePath}\``);
    lines.push('');
    
    if (library.headerComment) {
      lines.push('**Description:**');
      lines.push('');
      lines.push(library.headerComment);
      lines.push('');
    }
    
    if (library.modules.length > 0) {
      lines.push('**Available Modules/Functions:**');
      lines.push('');
      
      for (const module of library.modules) {
        lines.push(`#### ${module.name}`);
        lines.push('');
        lines.push('```openscad');
        lines.push(module.signature);
        lines.push('```');
        lines.push('');
        
        if (module.comment) {
          lines.push(module.comment);
          lines.push('');
        }
        
        if (module.parameters && module.parameters.length > 0) {
          lines.push('**Parameters:**');
          for (const param of module.parameters) {
            lines.push(`- \`${param}\``);
          }
          lines.push('');
        }
      }
    } else {
      lines.push('*No documented modules or functions found.*');
      lines.push('');
    }
    
    lines.push('---');
    lines.push('');
  }
  
  return lines.join('\n');
}

/**
 * Generate and save OpenSCAD libraries documentation as markdown
 */
export async function generateAndSaveLibrariesDocumentation(outputPath: string, workspaceRoot?: string): Promise<void> {
  const doc = await generateOpenSCADLibrariesDocumentation(workspaceRoot);
  const markdown = convertLibrariesDocumentationToMarkdown(doc);
  const outputUri = vscode.Uri.file(outputPath);
  await vscode.workspace.fs.writeFile(outputUri, new TextEncoder().encode(markdown));
}