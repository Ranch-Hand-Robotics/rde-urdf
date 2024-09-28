import * as os from "os";
import * as vscode from "vscode";
import * as child_process from "child_process";

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
  });
}

export function getPackages(): Promise<{ [name: string]: () => Promise<string> }> {
  return new Promise((resolve, reject) => async () => {
    const packages: { [name: string]: () => Promise<string> } = {};
    const { stdout } = child_process.exec("ros2 pkg list");

    if (!stdout) {
      return packages;
    }

    let chucks = "";
    for await (const chuck of stdout) {
        chucks += chuck;
    }

    chucks.split(os.EOL).map(((line) => {
        const packageName: string = line.trim();
        packages[packageName] = async (): Promise<string> => {
            const { stdout } = await child_process.exec(
                `ros2 pkg prefix --share ${packageName}`);
            if (!stdout) {
                return "";
            }

            let innerChucks = "";
            for await (const chuck of stdout) {
                innerChucks += chuck;
            }
            return innerChucks.trim();
        };
    }));

    return packages;
  });
}
