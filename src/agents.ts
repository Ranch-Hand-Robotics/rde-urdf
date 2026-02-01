import * as vscode from 'vscode';
import * as path from 'path';

let tracing: vscode.OutputChannel;
let extensionContext: vscode.ExtensionContext;

export function setTracing(trace: vscode.OutputChannel): void {
  tracing = trace;
}

export function setExtensionContext(context: vscode.ExtensionContext): void {
  extensionContext = context;
}

/**
 * Setup agents and skills by configuring workspace settings to point to extension directories
 */
export async function setupAgentsAndSkills(context: vscode.ExtensionContext): Promise<void> {
  try {
    const config = vscode.workspace.getConfiguration();

    // Get extension's .github directory paths
    const extensionAgentsPath = path.join(context.extensionPath, '.github', 'agents');
    const extensionSkillsPath = path.join(context.extensionPath, '.github', 'skills');

    // Configure chat.agentFilesLocations to point to extension's agents directory
    await config.update('chat.agentFilesLocations', [extensionAgentsPath], vscode.ConfigurationTarget.Workspace);
    tracing.appendLine(`Configured chat.agentFilesLocations to: ${extensionAgentsPath}`);

    // Configure chat.agentSkillsLocations to point to extension's skills directory
    await config.update('chat.agentSkillsLocations', [extensionSkillsPath], vscode.ConfigurationTarget.Workspace);
    tracing.appendLine(`Configured chat.agentSkillsLocations to: ${extensionSkillsPath}`);

    // Enable agent skills feature
    await config.update('chat.useAgentSkills', true, vscode.ConfigurationTarget.Workspace);
    tracing.appendLine('Enabled chat.useAgentSkills for workspace');

    // Enable instruction files
    await config.update('github.copilot.chat.codeGeneration.useInstructionFiles', true, vscode.ConfigurationTarget.Workspace);
    tracing.appendLine('Enabled github.copilot.chat.codeGeneration.useInstructionFiles for workspace');

    vscode.window.showInformationMessage('Robot Developer agents and skills workspace settings have been configured!');
  } catch (error) {
    const message = `Failed to setup agents and skills settings: ${error instanceof Error ? error.message : String(error)}`;
    vscode.window.showErrorMessage(message);
    tracing.appendLine(message);
  }
}

/**
 * Reset the agents and skills setup by clearing workspace settings
 */
export async function resetAgentsSetupState(): Promise<void> {
  if (!extensionContext) {
    vscode.window.showErrorMessage('Extension context not initialized');
    return;
  }

  try {
    const config = vscode.workspace.getConfiguration();

    // Clear workspace settings for agents and skills
    await config.update('chat.agentFilesLocations', undefined, vscode.ConfigurationTarget.Workspace);
    await config.update('chat.agentSkillsLocations', undefined, vscode.ConfigurationTarget.Workspace);
    await config.update('chat.useAgentSkills', undefined, vscode.ConfigurationTarget.Workspace);
    await config.update('github.copilot.chat.codeGeneration.useInstructionFiles', undefined, vscode.ConfigurationTarget.Workspace);

    vscode.window.showInformationMessage('Agents and skills workspace settings have been cleared.');
    tracing.appendLine('Agents and skills workspace settings cleared');
  } catch (error) {
    const message = `Failed to reset agents setup state: ${error instanceof Error ? error.message : String(error)}`;
    vscode.window.showErrorMessage(message);
    tracing.appendLine(message);
  }
}
