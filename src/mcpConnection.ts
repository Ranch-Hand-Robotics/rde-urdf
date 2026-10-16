import * as vscode from 'vscode';
import { randomUUID } from 'node:crypto';
import { UrdfMcpServer } from './mcp';

/** One controller per extension activation, never stored in a shared memento. */
export class WindowMcpConnection implements vscode.McpServerDefinitionProvider<vscode.McpHttpServerDefinition> {
  private readonly instanceId = randomUUID();
  private readonly changed = new vscode.EventEmitter<void>();
  readonly onDidChangeMcpServerDefinitions = this.changed.event;
  private server?: UrdfMcpServer;
  private pending: Promise<unknown> = Promise.resolve();
  private enabled = true;
  private disposed = false;
  private shutdown?: Promise<void>;
  private generation = 0;

  constructor(private readonly trace: vscode.OutputChannel) {}

  // VS Code derives custom-agent tool references (urdf/*) from this label.
  // Window identity belongs in transport metadata, not in the tool namespace.
  private readonly label = 'URDF';

  private enqueue<T>(operation: () => Promise<T>): Promise<T> {
    const result = this.pending.then(operation);
    // A failed start must not poison subsequent retries or shutdown.
    this.pending = result.catch(() => undefined);
    return result;
  }

  private async ensureStarted(): Promise<void> {
    if (this.disposed || !this.enabled || this.server) {
      return;
    }
    const server = new UrdfMcpServer(0, this.instanceId);
    await server.start();
    this.server = server;
    this.generation++;
    this.changed.fire();
  }

  start(): Promise<void> {
    return this.enqueue(async () => {
      if (this.disposed) {
        return;
      }
      this.enabled = true;
      await this.ensureStarted();
    });
  }

  stop(): Promise<void> {
    return this.enqueue(async () => {
      this.enabled = false;
      this.changed.fire();
      await this.server?.stop();
      this.server = undefined;
    });
  }

  private definition(includeCredentials = false): vscode.McpHttpServerDefinition {
    const connection = this.server!.getConnection();
    // VS Code can persist discovery definitions. Only resolve the secret at launch.
    const headers = includeCredentials ? connection.headers : { 'x-urdf-instance': this.instanceId };
    return new vscode.McpHttpServerDefinition(
      this.label, vscode.Uri.parse(connection.url), headers, `${this.instanceId}:${this.generation}`
    );
  }

  provideMcpServerDefinitions(): Promise<vscode.McpHttpServerDefinition[]> {
    return this.enqueue(async () => {
      if (this.disposed || !this.enabled) {
        return [];
      }
      await this.ensureStarted();
      return this.disposed ? [] : [this.definition()];
    });
  }

  resolveMcpServerDefinition(definition: vscode.McpHttpServerDefinition): Promise<vscode.McpHttpServerDefinition> {
    return this.enqueue(async () => {
      if (this.disposed || !this.enabled || definition.label !== this.label ||
          definition.headers?.['x-urdf-instance'] !== this.instanceId) {
        throw new Error('This URDF MCP definition is stopped or belongs to another VS Code window. Refresh MCP servers in this window.');
      }
      await this.ensureStarted();
      if (this.disposed) {
        throw new Error('This VS Code window is shutting down.');
      }
      // Do not trust a cached URI/header pair: always resolve to this window's live listener.
      return this.definition(true);
    });
  }

  register(context: vscode.ExtensionContext): void {
    if (vscode.lm?.registerMcpServerDefinitionProvider && vscode.McpHttpServerDefinition) {
      context.subscriptions.push(vscode.lm.registerMcpServerDefinitionProvider('URDF', this));
    } else {
      this.trace.appendLine('Window-bound URDF MCP discovery requires the VS Code MCP provider API. No unauthenticated fallback is available.');
    }
    context.subscriptions.push(
      vscode.workspace.onDidChangeWorkspaceFolders(() => this.changed.fire()),
      vscode.commands.registerCommand('urdf-editor.startMcpServer', () => this.start()),
      vscode.commands.registerCommand('urdf-editor.stopMcpServer', () => this.stop()),
      { dispose: () => { void this.dispose().catch(error => this.trace.appendLine(`MCP shutdown failed: ${error}`)); } }
    );
  }

  dispose(): Promise<void> {
    if (!this.shutdown) {
      this.disposed = true;
      this.shutdown = this.stop().finally(() => this.changed.dispose());
    }
    return this.shutdown;
  }
}