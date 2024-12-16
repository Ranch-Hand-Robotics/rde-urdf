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

export function xacro(filename: string): Promise<string> {
  return new Promise((resolve, reject) => {

    global.DOMParser = new JSDOM().window.DOMParser;

    const parser = new XacroParser();
    const xacroContents = fs.readFileSync( filename, { encoding: 'utf8' } );
    parser.parse( xacroContents ).then( result => {
      // Result is an XmlDom object, convert to string.
      const serializer = new XMLSerializer();
      const parsedXacro = serializer.serializeToString(result.documentElement) as string;
      resolve( parsedXacro );
    }).catch( error => {
      reject( error );
    });

    
      /*
      let processOptions = {
          cwd: vscode.workspace.rootPath,
          windowsHide: false,
      };

      let xacroCommand: string;
      if (process.platform === "win32") {
          xacroCommand = `cmd /c "xacro "${filename}""`;
      } else {
          xacroCommand = `bash --login -c "xacro '${filename}' && env"`;
      }

      child_process.exec(xacroCommand, processOptions, (error, stdout, _stderr) => {
          if (!error) {
              resolve(stdout);
          } else {
              reject(error);
          }
      });
    */
  });
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

export async function processXacro(filename: string, resolvePackagesFxn: (packageName :vscode.Uri) => string) : Promise<[string, string[]]> {
  return new Promise(async (resolve, reject) => {
    var packagesNotFound: string[] = [];
    var urdfText = "";
    try {
      urdfText = await xacro(filename);

      // Xacro may not add a <?xml> tag, so add it if it is not there.
      if (urdfText.indexOf("<?xml") === -1) {
          urdfText = '<?xml version="1.0"?>' + os.EOL + urdfText;
      }

      var packageMap = await getPackages();
      if (packageMap !== null) {
          // replace package://(x) with fully resolved paths
          var pattern =  /package:\/\/(.*?)\//g;
          var match;
          while (match = pattern.exec(urdfText)) {
              if (packageMap.hasOwnProperty(match[1]) === false) {
                  if (packagesNotFound.indexOf(match[1]) === -1) {
                      packagesNotFound.push(match[1]);
                  }
              } else {
                  var packagePath = await packageMap[match[1]];
                  if (packagePath.charAt(0)  === '/') {
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
                  let vsPath = vscode.Uri.file(normPath);
                  let newUri = resolvePackagesFxn(vsPath);

                  urdfText = urdfText.replace('package://' + match[1], newUri);
              }
          }
      }
    }
    catch (err: any) {
      rejects(err);
    }

    resolve([urdfText, packagesNotFound]);
  });
}
