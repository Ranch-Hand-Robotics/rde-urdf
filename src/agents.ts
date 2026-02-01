import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';

let tracing: vscode.OutputChannel;
let extensionContext: vscode.ExtensionContext;

export function setTracing(trace: vscode.OutputChannel): void {
  tracing = trace;
}

export function setExtensionContext(context: vscode.ExtensionContext): void {
  extensionContext = context;
}

/**
 * Recursively copy a directory and its contents
 */
export async function copyDirectory(srcDir: string, destDir: string): Promise<void> {
  // Create destination directory if it doesn't exist
  if (!fs.existsSync(destDir)) {
    fs.mkdirSync(destDir, { recursive: true });
  }

  const files = fs.readdirSync(srcDir);
  for (const file of files) {
    const srcPath = path.join(srcDir, file);
    const destPath = path.join(destDir, file);
    const stat = fs.statSync(srcPath);

    if (stat.isDirectory()) {
      await copyDirectory(srcPath, destPath);
    } else {
      fs.copyFileSync(srcPath, destPath);
    }
  }
}

/**
 * Check if agents and skills exist in the workspace
 */
export function doAgentsAndSkillsExist(workspacePath: string): boolean {
  const githubDir = path.join(workspacePath, '.github');
  const agentsDir = path.join(githubDir, 'agents');
  const skillsDir = path.join(githubDir, 'skills');
  
  return fs.existsSync(agentsDir) && fs.existsSync(skillsDir);
}

/**
 * Setup agents and skills by copying from extension to workspace and configuring workspace settings
 */
export async function setupAgentsAndSkills(context: vscode.ExtensionContext, workspacePath: string): Promise<void> {
  try {
    const githubDir = path.join(workspacePath, '.github');
    const agentsDir = path.join(githubDir, 'agents');
    const skillsDir = path.join(githubDir, 'skills');

    const extensionAgentsDir = path.join(context.extensionPath, '.github', 'agents');
    const extensionSkillsDir = path.join(context.extensionPath, '.github', 'skills');

    // Copy agents
    if (fs.existsSync(extensionAgentsDir)) {
      await copyDirectory(extensionAgentsDir, agentsDir);
      tracing.appendLine(`Copied agents from extension to ${agentsDir}`);
    }

    // Copy skills
    if (fs.existsSync(extensionSkillsDir)) {
      await copyDirectory(extensionSkillsDir, skillsDir);
      tracing.appendLine(`Copied skills from extension to ${skillsDir}`);
    }

    // Configure workspace settings to point to the relative paths within workspace
    const config = vscode.workspace.getConfiguration();
    
    // Use relative paths from workspace root
    await config.update('chat.agentFilesLocations', ['.github/agents'], vscode.ConfigurationTarget.Workspace);
    tracing.appendLine('Configured chat.agentFilesLocations to: .github/agents');
    
    await config.update('chat.agentSkillsLocations', ['.github/skills'], vscode.ConfigurationTarget.Workspace);
    tracing.appendLine('Configured chat.agentSkillsLocations to: .github/skills');
    
    await config.update('chat.useAgentSkills', true, vscode.ConfigurationTarget.Workspace);
    tracing.appendLine('Enabled chat.useAgentSkills for workspace');

    vscode.window.showInformationMessage('Robot Developer agents and skills have been set up in your workspace!');
  } catch (error) {
    const message = `Failed to setup agents and skills: ${error instanceof Error ? error.message : String(error)}`;
    vscode.window.showErrorMessage(message);
    tracing.appendLine(message);
  }
}

/**
 * Check if agents and skills are missing and offer to install them
 */
export async function checkAndOfferAgentsAndSkillsSetup(context: vscode.ExtensionContext): Promise<void> {
  const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
  if (!workspaceFolder) {
    // No workspace open, skip
    return;
  }

  // Check if already asked about this
  const neverAskAgainKey = 'urdf-editor.neverAskAgentsAndSkillsSetup';
  const dontAskThisSessionKey = 'urdf-editor.dontAskAgentsAndSkillsSetupThisSession';

  const userSettings = vscode.workspace.getConfiguration();
  if (userSettings.get(neverAskAgainKey)) {
    return; // User has set to never ask again
  }

  const globalState = context.globalState;
  if (globalState.get(dontAskThisSessionKey)) {
    return; // Already declined this session
  }

  // Check if agents and skills already exist
  if (doAgentsAndSkillsExist(workspaceFolder.uri.fsPath)) {
    return; // Already set up
  }

  // Show dialog with 10 second timeout
  const timeoutPromise = new Promise<string | undefined>((resolve) => {
    setTimeout(() => resolve(undefined), 10000);
  });

  const result = await Promise.race([
    vscode.window.showInformationMessage(
      'Install Robot Developer agents and skills for enhanced Copilot editing?',
      { modal: false },
      'Yes',
      'No',
      'Never ask again'
    ),
    timeoutPromise
  ]);

  if (result === 'Yes') {
    await setupAgentsAndSkills(context, workspaceFolder.uri.fsPath);
  } else if (result === 'No') {
    // Don't ask again this session
    await globalState.update(dontAskThisSessionKey, true);
  } else if (result === 'Never ask again') {
    // Don't ask again ever
    await userSettings.update(neverAskAgainKey, true, vscode.ConfigurationTarget.Global);
  }
}

/**
 * Reset the agents and skills setup state (debug/testing)
 */
export async function resetAgentsSetupState(): Promise<void> {
  if (!extensionContext) {
    vscode.window.showErrorMessage('Extension context not initialized');
    return;
  }

  try {
    const neverAskAgainKey = 'urdf-editor.neverAskAgentsAndSkillsSetup';
    const dontAskThisSessionKey = 'urdf-editor.dontAskAgentsAndSkillsSetupThisSession';

    // Clear global state
    await extensionContext.globalState.update(dontAskThisSessionKey, undefined);

    // Clear user settings
    const userSettings = vscode.workspace.getConfiguration();
    await userSettings.update(neverAskAgainKey, undefined, vscode.ConfigurationTarget.Global);
    
    // Clear workspace settings for agents and skills
    await userSettings.update('chat.agentFilesLocations', undefined, vscode.ConfigurationTarget.Workspace);
    await userSettings.update('chat.agentSkillsLocations', undefined, vscode.ConfigurationTarget.Workspace);
    await userSettings.update('chat.useAgentSkills', undefined, vscode.ConfigurationTarget.Workspace);

    vscode.window.showInformationMessage('Agents setup state has been reset. Run extension activation to see the dialog again.');
    tracing.appendLine('Agents setup state reset');
  } catch (error) {
    const message = `Failed to reset agents setup state: ${error instanceof Error ? error.message : String(error)}`;
    vscode.window.showErrorMessage(message);
    tracing.appendLine(message);
  }
}
