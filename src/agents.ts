import * as vscode from 'vscode';

let tracing: vscode.OutputChannel;
let extensionContext: vscode.ExtensionContext;

export function setTracing(trace: vscode.OutputChannel): void {
  tracing = trace;
}

export function setExtensionContext(context: vscode.ExtensionContext): void {
  extensionContext = context;
}
