import * as os from "os";
import * as vscode from "vscode";
//import * as child_process from "child_process";
import { XacroParser } from 'xacro-parser';
import {XMLSerializer} from 'xmldom';
import { JSDOM } from 'jsdom';
import * as path from "path";
import * as fs from "fs";
import { string } from "yaml/dist/schema/common/string";
import { rejects } from "assert";
import { tracing } from "./extension";
import { env } from "process";




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

export async function getPackages(): Promise<Map<string, string>> {
    var packages = new Map<string, string>();

    // For this project, we're focusing on URDF packages within the same workspace. 
    // We will not use ros2 cli to list packages
    // Iterate over the top level directories in the 'src' directory, looking for package.xml
    // Extract the package name
    // Identify the package path
    // assume folders relative to this path are where resources are located.
    // for example if you have a package called 'robot1', with a directory called 'meshes' containing a file called 'robot1.stl', then 
    // the return from this function will be the root of robot1's package directory

    // Get the workspace path from vscode api:

    const workspaceFolders = vscode.workspace.workspaceFolders;
    if (!workspaceFolders) {
      return packages;
    }

    // for each workspace folder, look for package.xml files
    for (const workspaceFolder of workspaceFolders) {
      const packageXmlFiles = await vscode.workspace.findFiles(
        new vscode.RelativePattern(workspaceFolder, "**/package.xml")
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

        packages.set(packageName, path.dirname(packageXmlFile.fsPath));
        };
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

    } catch (err: any) {
      reject(err);
      return;
    }

    resolve([urdfText, packagesNotFound]);
  });
}
