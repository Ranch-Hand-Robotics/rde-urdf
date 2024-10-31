import * as os from "os";
import * as vscode from "vscode";
import * as child_process from "child_process";
import { XacroParser } from 'xacro-parser';
import {XMLSerializer} from 'xmldom';
import { JSDOM } from 'jsdom';
import path = require("path");
import fs = require("fs");
import { string } from "yaml/dist/schema/common/string";

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

export function xacro(filename: string): Promise<any> {
  return new Promise((resolve, reject) => {

    global.DOMParser = new JSDOM().window.DOMParser;

    const parser = new XacroParser();
    const xacroContents = fs.readFileSync( filename, { encoding: 'utf8' } );
    parser.parse( xacroContents ).then( result => {
      // Result is an XmlDom object, convert to string.
      const serializer = new XMLSerializer();
      const parsedXacro = serializer.serializeToString(result.documentElement);
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
