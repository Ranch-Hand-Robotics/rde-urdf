import * as os from "os";
import * as vscode from "vscode";
//import * as child_process from "child_process";
import { XacroParser } from 'xacro-parser';
import {XMLSerializer} from 'xmldom';
import { JSDOM } from 'jsdom';
import * as path from "path";
import * as fs from "fs";
import { tracing } from "./extension";




export function getNonce() {
  let text = "";
  const possible = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  for (let i = 0; i < 32; i++) {
    text += possible.charAt(Math.floor(Math.random() * possible.length));
  }
  return text;
}

export function getUri(webview: vscode.Webview, extensionUri: vscode.Uri, pathList: string[]) {
  return webview.asWebviewUri(vscode.Uri.joinPath(extensionUri, ...pathList));
}

/**
 * URDF/Xacro Processing Pipeline:
 * 
 * The processXacro function implements a streamlined processing pipeline that:
 * 1. Uses XacroParser to parse the xacro file directly
 * 2. Custom getFileContents function detects and resolves $(find package) idioms
 * 3. Package:// URIs are resolved after xacro processing is complete
 * 4. Final URDF content has all package references resolved to webview URIs
 * 
 * This approach ensures that:
 * - XacroParser handles macro expansion and processing first
 * - $(find package) idioms are resolved during file loading but content is not parsed as xacro
 * - Package:// URI mapping occurs after xacro processing to handle generated content
 * - File includes and mesh references resolve correctly during and after macro expansion
 */

/**
 * Converts ${find package} idiom to package:// format.
 * This is commonly used in ROS launch files and xacro files.
 * @param text The input text containing ${find package} patterns
 * @returns The text with ${find package} converted to package:// format
 */
export function convertFindToPackageUri(text: string): string {
  // Pattern to match ${find package_name} where package_name can contain letters, numbers, underscores, and hyphens
  const findPattern = /\$\(find\s+([a-zA-Z0-9_-]+)\)/g;
  
  return text.replace(findPattern, (match, packageName) => {
    return `package://${packageName}`;
  });
}

/**
 * Processes a file's content to convert ${find package} idiom to package:// format.
 * This function reads the file, performs the conversion, and returns the modified content.
 * @param filePath The path to the file to process
 * @returns Promise resolving to the converted file content
 */
export async function convertFindToPackageUriInFile(filePath: string): Promise<string> {
  try {
    const fileContent = fs.readFileSync(filePath, { encoding: 'utf8' });
    return convertFindToPackageUri(fileContent);
  } catch (error) {
    throw new Error(`Failed to process file ${filePath}: ${error}`);
  }
}

/**
 * Discovers ROS packages from multiple sources and returns a map of package names to their filesystem paths.
 * 
 * This function searches for ROS packages in the following order of precedence:
 * 1. **Workspace folders**: Current VS Code workspace directories (highest precedence)
 * 2. **ROS distro directory**: System-installed ROS packages from configured distro
 * 3. **User-specified search paths**: Additional directories configured via `urdf-editor.PackageSearchPaths`
 * 
 * Package discovery works by scanning each source directory for `package.xml` files and extracting
 * the package name from the `<name>` element. Workspace packages take precedence over ROS distro
 * packages to allow for local development overrides.
 * 
 * **Configuration Settings Used:**
 * - `ROS2.distro`: ROS distribution name (e.g., "kilted", "humble")
 * - `ROS2.pixiRoot`: Base path for pixi-managed ROS installations
 * - `urdf-editor.PackageSearchPaths`: Array of additional directories to search
 * 
 * **Variable Substitution:**
 * - `${workspaceFolder}` in `PackageSearchPaths` is replaced with the workspace root path
 * 
 * @returns Promise resolving to a Map where keys are package names and values are absolute paths
 *          to the package directories containing the package.xml files
 * 
 * @example
 * ```typescript
 * const packages = await getPackages();
 * console.log(packages.get('robot_state_publisher')); // "/opt/ros/kilted/share/robot_state_publisher"
 * ```
 */
export async function getPackages(): Promise<Map<string, string>> {
    var packages = new Map<string, string>();

    // Get the workspace path from vscode api:
    const workspaceFolders = vscode.workspace.workspaceFolders;
    if (!workspaceFolders) {
      return packages;
    }

    // Collect all directories to scan (workspace folders + ROS distro if configured)
    const directoriesToScan: vscode.WorkspaceFolder[] = [...workspaceFolders];

    // === ROS Distro Package Discovery ===
    // Check for ROS distro configuration
    const rosConfig = vscode.workspace.getConfiguration('ROS2');
    const distro = rosConfig.get<string>('distro');
    const pixiRoot = rosConfig.get<string>('pixiRoot');
    
    if (distro) {
      let rosSharePath: string;
      
      if (pixiRoot) {
        // Use pixiRoot directly as the base path
        rosSharePath = path.join(pixiRoot, 'share');
      } else {
        // Fall back to standard ROS installation path
        rosSharePath = path.join('/opt/ros', distro, 'share');
      }
      
      // Check if the ROS share directory exists
      if (fs.existsSync(rosSharePath)) {
        // Create a virtual workspace folder for the ROS distro
        const rosWorkspaceFolder: vscode.WorkspaceFolder = {
          uri: vscode.Uri.file(rosSharePath),
          name: `ROS ${distro}`,
          index: workspaceFolders.length
        };
        directoriesToScan.push(rosWorkspaceFolder);
      }
    }

    // === User-Specified Package Search Paths ===
    // Add user-specified package search paths
    const urdfConfig = vscode.workspace.getConfiguration('urdf-editor');
    const packageSearchPaths = urdfConfig.get<string[]>('PackageSearchPaths') || [];
    
    for (const searchPath of packageSearchPaths) {
      // Resolve ${workspaceFolder} variable if present
      let resolvedPath = searchPath;
      if (resolvedPath.includes('${workspaceFolder}')) {
        const workspaceRoot = workspaceFolders[0]?.uri.fsPath || '';
        resolvedPath = resolvedPath.replace('${workspaceFolder}', workspaceRoot);
      }
      
      // Check if the directory exists
      if (fs.existsSync(resolvedPath)) {
        // Create a virtual workspace folder for the search path
        const searchFolder: vscode.WorkspaceFolder = {
          uri: vscode.Uri.file(resolvedPath),
          name: `Package Search: ${path.basename(resolvedPath)}`,
          index: directoriesToScan.length
        };
        directoriesToScan.push(searchFolder);
      }
    }

    // === Package Discovery ===
    // Scan all directories (workspace + ROS distro) for package.xml files
    for (const folder of directoriesToScan) {
      const packageXmlFiles = await vscode.workspace.findFiles(
        new vscode.RelativePattern(folder, "**/package.xml")
      );

      for (const packageXmlFile of packageXmlFiles) {
        const packageXml = await vscode.workspace.fs.readFile(packageXmlFile);
        const packageXmlString = new TextDecoder().decode(packageXml);
        const packageXmlDom = new JSDOM(packageXmlString);
        const packageXmlDocument = packageXmlDom.window.document;

        const packageName = packageXmlDocument.querySelector("name")?.textContent;
        if (!packageName) {
          continue;
        }

        // For ROS distro packages, only add if not already found in workspace
        const isRosDistroPackage = folder.name.startsWith('ROS ');
        if (!isRosDistroPackage || !packages.has(packageName)) {
          packages.set(packageName, path.dirname(packageXmlFile.fsPath));
        }
      }
    }

    return packages;
}

/**
 * Processes URDF or xacro content to convert ${find package} patterns.
 * This is useful for processing content that may come from various sources.
 * @param content The URDF/xacro content to process
 * @returns The processed content with ${find package} converted to package:// format
 */
export function processUrdfContent(content: string): string {
  return convertFindToPackageUri(content);
}

/**
 * Resolves package:// URIs to actual file paths after xacro processing.
 * This function maps package URIs to resolved webview URIs.
 * 
 * @param urdfContent The URDF content containing package:// URIs
 * @param packageMap Map of package names to their filesystem paths
 * @param resolvePackagesFxn Function to resolve package URIs to webview URIs
 * @param packagesNotFound Array to collect missing package names
 * @returns The URDF content with package:// URIs resolved to webview URIs
 */
function resolvePackageUris(
  urdfContent: string,
  packageMap: Map<string, string>,
  resolvePackagesFxn: (packageUri: vscode.Uri) => string,
  packagesNotFound: string[]
): string {
  if (!packageMap || packageMap.size === 0) {
    return urdfContent;
  }

  const pattern = /package:\/\/(.*?)\//g;
  let match;
  let resolvedContent = urdfContent;

  while ((match = pattern.exec(urdfContent)) !== null) {
    const packageName = match[1];
    
    if (!packageMap.has(packageName)) {
      if (!packagesNotFound.includes(packageName)) {
        packagesNotFound.push(packageName);
      }
    } else {
      const packagePath = packageMap.get(packageName)!;
      let normalizedPath = packagePath;
      
      // Handle absolute paths
      if (normalizedPath.charAt(0) === '/') {
        normalizedPath = normalizedPath.substring(1);
      }
      
      // Normalize path and convert backslashes to forward slashes on Windows
      normalizedPath = path.normalize(normalizedPath);
      if (os.platform() === 'win32') {
        normalizedPath = normalizedPath.replace(/\\/g, '/');
      }
      
      const vsPath = vscode.Uri.file(normalizedPath);
      const newUri = resolvePackagesFxn(vsPath);

      // Replace all occurrences of this package URI
      resolvedContent = resolvedContent.replace(
        new RegExp(`package://${packageName}/`, 'g'),
        newUri + '/'
      );
    }
  }

  return resolvedContent;
}

/**
 * Checks if a path is a relative path (not absolute, not a URL with protocol)
 * @param filePath The file path to check
 * @returns true if the path is relative
 */
function isRelativePath(filePath: string): boolean {
  if (!filePath || filePath.trim() === '') {
    return false;
  }
  
  // Check for protocols (package://, file://, http://, https://, etc.)
  if (filePath.includes('://')) {
    return false;
  }
  
  // Check for absolute paths
  // Unix-style absolute path starts with /
  if (filePath.startsWith('/')) {
    return false;
  }
  
  // Windows-style absolute path (e.g., C:\, D:\)
  if (filePath.length >= 2 && filePath[1] === ':') {
    return false;
  }
  
  // If none of the above, it's a relative path
  return true;
}

/**
 * Resolves relative file paths in URDF content to absolute paths.
 * This function finds mesh filename attributes with relative paths and converts them
 * to absolute paths based on the URDF file's directory.
 * 
 * @param urdfContent The URDF content that may contain relative paths
 * @param urdfFilePath The absolute path to the URDF file being processed
 * @param resolvePathFxn Function to convert absolute paths to webview URIs
 * @returns The URDF content with relative paths resolved to webview URIs
 */
function resolveRelativePaths(
  urdfContent: string,
  urdfFilePath: string,
  resolvePathFxn: (absolutePath: vscode.Uri) => string
): string {
  // Get the directory containing the URDF file
  const urdfDir = path.dirname(urdfFilePath);
  
  // Pattern to match filename attributes in mesh, texture, etc.
  // Matches: filename="some/path/file.ext"
  const filenamePattern = /filename=["']([^"']+)["']/g;
  
  let resolvedContent = urdfContent;
  let match;
  
  // Reset regex state
  filenamePattern.lastIndex = 0;
  
  while ((match = filenamePattern.exec(urdfContent)) !== null) {
    const originalPath = match[1];
    
    // Check if this is a relative path
    if (isRelativePath(originalPath)) {
      // Resolve the relative path to an absolute path
      const absolutePath = path.resolve(urdfDir, originalPath);
      
      // Normalize path separators for the platform
      let normalizedPath = path.normalize(absolutePath);
      
      // Convert to webview URI
      const vsPath = vscode.Uri.file(normalizedPath);
      const webviewUri = resolvePathFxn(vsPath);
      
      // Replace this occurrence in the content
      // Use a more specific pattern to avoid replacing unrelated matches
      const searchPattern = `filename="${originalPath}"`;
      const replacement = `filename="${webviewUri}"`;
      
      // Only replace if the pattern exists
      if (resolvedContent.includes(searchPattern)) {
        resolvedContent = resolvedContent.replace(searchPattern, replacement);
      } else {
        // Try with single quotes
        const searchPatternSingleQuote = `filename='${originalPath}'`;
        const replacementSingleQuote = `filename='${webviewUri}'`;
        if (resolvedContent.includes(searchPatternSingleQuote)) {
          resolvedContent = resolvedContent.replace(searchPatternSingleQuote, replacementSingleQuote);
        }
      }
    }
  }
  
  return resolvedContent;
}

/**
 * Processes a xacro file with package resolution.
 * This function handles the processing pipeline:
 * 1. Uses XacroParser to parse the xacro file with custom getFileContents
 * 2. Maps package:// URIs to resolved paths after xacro processing
 * 3. Returns the final URDF content and any missing packages
 * 
 * @param filename The path to the xacro file to process
 * @param resolvePackagesFxn Function to resolve package URIs to webview URIs
 * @returns Promise resolving to [urdfText, packagesNotFound]
 */
export async function processXacro(filename: string, resolvePackagesFxn: (packageName: vscode.Uri) => string): Promise<[string, string[]]> {
  return new Promise(async (resolve, reject) => {
    const packagesNotFound: string[] = [];
    let urdfText = "";

    try {
      // Read the file content
      const xacroContents = fs.readFileSync(filename, { encoding: 'utf8' });
      
      // Get package map for resolving package:// URIs and $(find) idioms
      const packageMap = await getPackages();

      // Setup XacroParser with custom getFileContents
      global.DOMParser = new JSDOM().window.DOMParser;
      const parser = new XacroParser();

      parser.getFileContents = async (filePath: string): Promise<string> => {
        if (!filePath || filePath.trim() === "") {
          return "";
        }
        
        try {
          let resolvedPath = filePath;

          // Handle vscode-resource URLs
          if (resolvedPath.startsWith("https://file%2B.vscode-resource.vscode-cdn.net/")) {
            resolvedPath = resolvedPath.replace("https://file%2B.vscode-resource.vscode-cdn.net/", "");
          }

          // Handle package:// protocol
          if (resolvedPath.startsWith("package://")) {
            const packagePattern = /^package:\/\/([^/]+)(\/.*)?$/;
            const packageMatch = packagePattern.exec(resolvedPath);
            
            if (packageMatch && packageMatch.length >= 2) {
              const packageName = packageMatch[1];
              const resourcePath = packageMatch[2] || '';
              
              if (!packageMap.has(packageName)) {
                if (!packagesNotFound.includes(packageName)) {
                  packagesNotFound.push(packageName);
                }
                return "";
              }
              
              const packageBasePath = packageMap.get(packageName)!;
              resolvedPath = path.join(packageBasePath, resourcePath);
            }
          }
          // Handle relative paths - resolve relative to the main URDF/Xacro file
          else if (isRelativePath(resolvedPath)) {
            const baseDir = path.dirname(filename);
            resolvedPath = path.resolve(baseDir, resolvedPath);
          }

          // Normalize and decode the file path
          let normalizedPath = path.normalize(resolvedPath);
          normalizedPath = decodeURIComponent(normalizedPath);

          // Read and return file contents (not parsed as xacro)
          return fs.readFileSync(normalizedPath, { encoding: 'utf8' });
          
        } catch (error) {
          throw new Error(`File not found: ${filePath}`);
        }
      };

      parser.rospackCommands =
      {
        find: (...args:string[]): string => {
          let packageName = args[0] as string;
          if (!packageName || packageName.trim() === "") {
            return "";
          }

          if (!packageMap.has(packageName)) {
            if (!packagesNotFound.includes(packageName)) {
              packagesNotFound.push(packageName);
            }
            return "";
          }
          return packageMap.get(packageName)!;
        },

        eval: (...args: string[]): string => {
          // This function is not used in the current implementation
          // but can be extended to handle other rospack commands if needed

          tracing.appendLine(`rospack eval is not supported currently, called with args: ${args.join(', ')}. If you'd like to see this implemented, +1 https://github.com/Ranch-Hand-Robotics/rde-urdf/issues/43, or submit a pull request.`);

          return "";
        },

        dirname: (filePath: string): string => {
          if (!filePath || filePath.trim() === "") {
            return "";
          }
          return path.dirname(filePath);
        },

        anon: (arg: string): string => {
          // return random string for anonymous names
          return getNonce();
        },

        optenv: (...args : Array<String>): string => {
          // This function is not used in the current implementation
          // but can be extended to handle other rospack commands if needed

          tracing.appendLine(`rospack optenv is not supported currently. If you'd like to see this implemented, +1 https://github.com/Ranch-Hand-Robotics/rde-urdf/issues/45`);
          if (args.length < 2) {
            return "";
          }
          return args.slice(1).join("");
        },

        env: (env: string): string => {
          // Get the environment variable if it exists, otherwise return empty string
          return env[env.toString()] !== undefined ? env[env.toString()].toString() : "";
        }
      };

      // Add XML declaration if missing
      if (urdfText.indexOf("<?xml") === -1) {
        urdfText = '<?xml version="1.0"?>' + os.EOL + urdfText;
      }

      // Parse the xacro content
      const result = await parser.parse(xacroContents);
      
      // Convert result to string
      const serializer = new XMLSerializer();
      urdfText = serializer.serializeToString(result.documentElement) as string;

      // Process package:// URIs in the final URDF content
      urdfText = resolvePackageUris(urdfText, packageMap, resolvePackagesFxn, packagesNotFound);
      
      // Process relative paths in the final URDF content
      urdfText = resolveRelativePaths(urdfText, filename, resolvePackagesFxn);

    } catch (err: any) {
      reject(err);
      return;
    }

    resolve([urdfText, packagesNotFound]);
  });
}
