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
      const stat = await fs.promises.stat(libPath);
      if (stat.isDirectory()) {
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
  const fullPath = path.join(basePath, relativePath);
  
  try {
    const entries = await fs.promises.readdir(fullPath, { withFileTypes: true });
    
    for (const entry of entries) {
      const entryPath = path.join(relativePath, entry.name);
      const entryPathUnix = path.posix.join(relativePath, entry.name);
      const fullEntryPath = path.join(fullPath, entry.name);
      const virtualPath = path.posix.join(virtualRoot, entryPathUnix);
      
      if (entry.isDirectory()) {
        // Recursively load subdirectories
        await loadLibraryDirectory(instance, basePath, entryPath, virtualRoot, trace);
      } else if (entry.isFile()) {
        try {
          instance.FS.mkdir(path.dirname(virtualPath));
        } catch {
          // ignore
        }

        try {
          const content = await fs.promises.readFile(fullEntryPath);
          instance.FS.writeFile(virtualPath, content);
        } catch {
          trace.appendLine(`Failed to load file ${entryPath}`);
        }
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
 * Convert an OpenSCAD file to STL format using openscad-wasm
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
    const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
    
    // Load OpenSCAD libraries
    trace.appendLine(`Loading OpenSCAD libraries...`);
    const libraryPaths = await getAllOpenSCADLibraryPaths(workspaceRoot);
    if (libraryPaths.length > 0) {
      trace.appendLine(`Found ${libraryPaths.length} library paths: ${libraryPaths.join(', ')}`);
      await loadLibraryFiles(instance, libraryPaths, trace);
    } else {
      trace.appendLine(`No OpenSCAD library paths found`);
    }

    const basename = path.basename(scadFilePath, '.scad');
    const dir = path.dirname(scadFilePath);
    const stlPath = path.join(dir, `${basename}.stl`);
    
    trace.appendLine(`Converting OpenSCAD to STL: ${stlPath}`);

    const scadText = await fs.promises.readFile(scadFilePath, 'utf8');
    
    // Write the main SCAD file to the virtual filesystem
    instance.FS.writeFile('/input.scad', scadText);
    
    // Add library search paths to OpenSCAD arguments
    const args = ['-o', '/output.stl', '/input.scad'];

    // Run OpenSCAD with library paths
    instance.callMain(args);
    
    // Read the output STL from the virtual filesystem
    const stlContent = instance.FS.readFile('/output.stl', { encoding: 'binary' });

    // Use the filesystem API directly
    await fs.promises.writeFile(stlPath, stlContent, 'binary');
    
    trace.appendLine(`OpenSCAD conversion completed successfully: ${stlPath}`);
    return stlPath;
  } catch (error) {
    trace.appendLine(`OpenSCAD conversion failed`);
    vscode.window.showErrorMessage(`Failed to convert OpenSCAD file, check output for more info`);
    return null;
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
  const fullPath = path.join(basePath, relativePath);
  
  try {
    const entries = await fs.promises.readdir(fullPath, { withFileTypes: true });
    
    for (const entry of entries) {
      const entryRelativePath = path.join(relativePath, entry.name);
      const fullEntryPath = path.join(fullPath, entry.name);
      
      if (entry.isDirectory()) {
        // Recursively process subdirectories
        await processLibraryDirectory(basePath, entryRelativePath, libraries);
      } else if (entry.isFile() && entry.name.endsWith('.scad')) {
        try {
          const content = await fs.promises.readFile(fullEntryPath, 'utf8');
          const headerComment = extractHeaderComment(content);
          const modules = extractModules(content);
          
          libraries.push({
            filePath: fullEntryPath,
            relativePath: entryRelativePath,
            headerComment,
            modules
          });
        } catch (error) {
          // Skip files that can't be read
          console.warn(`Failed to process ${fullEntryPath}: ${error}`);
        }
      }
    }
  } catch (error) {
    console.warn(`Failed to read directory ${fullPath}: ${error}`);
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
  await fs.promises.writeFile(outputPath, markdown, 'utf8');
}