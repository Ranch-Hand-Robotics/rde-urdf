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
 * The processXacro function implements a complete processing pipeline that handles:
 * 1. $(find package) idiom conversion to package:// format
 * 2. Package resolution using workspace package.xml files  
 * 3. Path resolution to webview-compatible URIs
 * 4. Xacro macro expansion and processing
 * 
 * This unified approach ensures that:
 * - $(find package) conversion happens before xacro parsing, as xacro 
 *   parameters and includes may contain find expressions
 * - Package resolution happens on the raw content before xacro processing,
 *   so that file includes and mesh references resolve correctly during macro expansion
 * - All preprocessing is completed before xacro parsing to ensure the parser
 *   works with valid, resolved file paths and content
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

        packages[packageName] = path.dirname(packageXmlFile.fsPath);
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
 * Processes a xacro file with complete package resolution and find idiom conversion.
 * This function handles the full pipeline:
 * 1. Reads the xacro file content
 * 2. Converts $(find package) idioms to package:// format
 * 3. Resolves package:// URIs to actual file paths
 * 4. Parses the xacro content with resolved paths
 * 5. Returns the final URDF content and any missing packages
 * 
 * @param filename The path to the xacro file to process
 * @param resolvePackagesFxn Function to resolve package URIs to webview URIs
 * @returns Promise resolving to [urdfText, packagesNotFound]
 */
export async function processXacro(filename: string, resolvePackagesFxn: (packageName :vscode.Uri) => string) : Promise<[string, string[]]> {
  return new Promise(async (resolve, reject) => {
    var packagesNotFound: string[] = [];
    var urdfText = "";
    try {
      // Read the file content
      let xacroContents = fs.readFileSync(filename, { encoding: 'utf8' });
      
      // Convert $(find package) idiom to package:// format before parsing
      xacroContents = convertFindToPackageUri(xacroContents);
      
      // Get package map for resolving package:// URIs
      var packageMap = await getPackages();
      
      // Replace package:// URIs with resolved paths in the raw xacro content
      // package://package_name/path/to/resource should be replaced with the actual path to the resource
      // For example, package://my_package/meshes/robot.stl where my_package is located in /home/user/ros2_ws/src/mypackage, the url should be replaced with /home/user/ros2_ws/src/my_package/meshes/robot.stl
      // on Windows, it should be replaced with C:\Users\user\ros2_ws\src\my_package\meshes\robot.stl
      if (packageMap !== null) {
          var pattern = /package:\/\/(.*?)\//g;
          var match;
          while (match = pattern.exec(xacroContents)) {
              if (packageMap.hasOwnProperty(match[1]) === false) {
                  if (packagesNotFound.indexOf(match[1]) === -1) {
                      packagesNotFound.push(match[1]);
                  }
              } else {
                  var packagePath = await packageMap[match[1]];
                  if (packagePath.charAt(0) === '/') {
                      // inside of mesh re \source, the loader attempts to concatinate the base uri with the new path. It first checks to see if the
                      // base path has a /, if not it adds it.
                      // We are attempting to use a protocol handler as the base path - which causes this to fail.
                      // basepath - vscode-webview-resource:
                      // full path - /home/test/ros
                      // vscode-webview-resource://home/test/ros.
                      // It should be vscode-webview-resource:/home/test/ros.
                      // So remove the first char.

                      packagePath = packagePath.substr(1);
                  }
                    let normPath = path.normalize(packagePath);
                    // Convert backslashes to forward slashes on Windows
                    if (os.platform() === 'win32') {
                      normPath = normPath.replace(/\\/g, '/');
                    }
                  let vsPath = vscode.Uri.file(normPath);
                  let newUri = resolvePackagesFxn(vsPath);

                  // Replace all occurrences in the content
                  xacroContents = xacroContents.replace(new RegExp('package://' + match[1], 'g'), newUri);
              }
          }
      }

      // Now parse the preprocessed xacro content
      global.DOMParser = new JSDOM().window.DOMParser;
      const parser = new XacroParser();

      parser.getFileContents = async (filePath: string): Promise<string> => {
        if (!filePath || filePath.trim() === "") {
          return "";
        }
        
        // Check if the file exists
        try {
          // detect if this starts with "https://file%2B.vscode-resource.vscode-cdn.net/ then remove the prefix and convert to a file URI
          if (filePath.startsWith("https://file%2B.vscode-resource.vscode-cdn.net/")) {
            // Remove the prefix
            filePath = filePath.replace("https://file%2B.vscode-resource.vscode-cdn.net/", "");
          }

          // Handle package:// protocol
          if (filePath.startsWith("package://")) {
            const packagePattern = /^package:\/\/([^/]+)(\/.*)?$/;
            const match = packagePattern.exec(filePath);
            
            if (match && match.length >= 2) {
              const packageName = match[1];
              const resourcePath = match[2] || '';
              
              if (!packageMap.hasOwnProperty(packageName)) {
                if (!packagesNotFound.includes(packageName)) {
                  packagesNotFound.push(packageName);
                }
                return "";
              }
              
              const packageBasePath = packageMap[packageName];
              filePath = path.join(packageBasePath, resourcePath);
            }
          }

          // Resolve the file path to a URI
          let filename = path.normalize(filePath);

          // uri decode filename, as windows filenames may contain encoded characters
          filename = decodeURIComponent(filename);

          let [content, missingPackages] = await processXacro(filename, resolvePackagesFxn);
          // If there are missing packages from the included file, add them to our list
          if (missingPackages && missingPackages.length > 0) {
            for (const pkg of missingPackages) {
              if (!packagesNotFound.includes(pkg)) {
          packagesNotFound.push(pkg);
              }
            }
          }
          return content;
          
        } catch (error) {
          // If the file does not exist, throw an error
          throw new Error(`File not found: ${filePath}`);
        }
      };
      
      const result = await parser.parse(xacroContents);
      // Result is an XmlDom object, convert to string.
      const serializer = new XMLSerializer();
      urdfText = serializer.serializeToString(result.documentElement) as string;

      // Xacro may not add a <?xml> tag, so add it if it is not there.
      if (urdfText.indexOf("<?xml") === -1) {
          urdfText = '<?xml version="1.0"?>' + os.EOL + urdfText;
      }
    }
    catch (err: any) {
      reject(err);
      return;
    }

    resolve([urdfText, packagesNotFound]);
  });
}
