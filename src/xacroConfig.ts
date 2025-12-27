/**
 * Xacro Configuration Management
 * 
 * This module handles loading and managing xacro arguments and environment variables
 * from the .vscode/xacro.json configuration file.
 */

import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import { minimatch } from 'minimatch';
import { tracing } from './extension';

/**
 * Configuration for a specific xacro file or pattern
 */
export interface XacroFileConfig {
    args?: { [key: string]: string | number | boolean };
    env?: { [key: string]: string };
}

/**
 * Root configuration structure matching the spec
 */
export interface XacroConfig {
    version: string;
    [filePattern: string]: string | XacroFileConfig;
}

const CONFIG_FILE_NAME = 'xacro.json';

/**
 * Gets the path to the xacro.json configuration file
 */
export function getConfigFilePath(workspaceFolder?: vscode.WorkspaceFolder): string | null {
    const folder = workspaceFolder || vscode.workspace.workspaceFolders?.[0];
    if (!folder) {
        return null;
    }
    return path.join(folder.uri.fsPath, '.vscode', CONFIG_FILE_NAME);
}

/**
 * Loads the xacro configuration from .vscode/xacro.json
 */
export async function loadXacroConfig(workspaceFolder?: vscode.WorkspaceFolder): Promise<XacroConfig | null> {
    const configPath = getConfigFilePath(workspaceFolder);
    if (!configPath || !fs.existsSync(configPath)) {
        return null;
    }

    try {
        const content = fs.readFileSync(configPath, 'utf8');
        const config = JSON.parse(content) as XacroConfig;
        return config;
    } catch (error) {
        tracing.appendLine(`Failed to load xacro configuration: ${error instanceof Error ? error.message : String(error)}`);
        return null;
    }
}

/**
 * Saves the xacro configuration to .vscode/xacro.json
 */
export async function saveXacroConfig(config: XacroConfig, workspaceFolder?: vscode.WorkspaceFolder): Promise<boolean> {
    const configPath = getConfigFilePath(workspaceFolder);
    if (!configPath) {
        return false;
    }

    // Ensure .vscode directory exists
    const vscodePath = path.dirname(configPath);
    if (!fs.existsSync(vscodePath)) {
        fs.mkdirSync(vscodePath, { recursive: true });
    }

    try {
        const content = JSON.stringify(config, null, 4);
        fs.writeFileSync(configPath, content, 'utf8');
        return true;
    } catch (error) {
        tracing.appendLine(`Failed to save xacro configuration: ${error instanceof Error ? error.message : String(error)}`);
        return false;
    }
}

/**
 * Finds the configuration for a specific file path by matching against patterns
 * Supports wildcards like ** and *.xacro
 */
export function findConfigForFile(filePath: string, config: XacroConfig | null): XacroFileConfig | null {
    if (!config) {
        return null;
    }

    // Normalize the file path
    const normalizedPath = path.normalize(filePath).replace(/\\/g, '/');

    // Try exact match first
    for (const pattern in config) {
        if (pattern === 'version') {
            continue;
        }

        // Resolve ${workspaceFolder} in pattern
        let resolvedPattern = pattern;
        if (pattern.includes('${workspaceFolder}')) {
            const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath || '';
            resolvedPattern = pattern.replace(/\$\{workspaceFolder\}/g, workspaceRoot);
        }

        // Normalize pattern
        resolvedPattern = path.normalize(resolvedPattern).replace(/\\/g, '/');

        // Try exact match
        if (normalizedPath === resolvedPattern) {
            return config[pattern] as XacroFileConfig;
        }

        // Try pattern matching with minimatch
        if (minimatch(normalizedPath, resolvedPattern)) {
            return config[pattern] as XacroFileConfig;
        }

        // Try matching just the filename part for patterns like "*.xacro"
        const fileName = path.basename(normalizedPath);
        const patternFileName = path.basename(resolvedPattern);
        if (minimatch(fileName, patternFileName)) {
            return config[pattern] as XacroFileConfig;
        }
    }

    return null;
}

/**
 * Detects if a file contains xacro:arg or $(arg ...) or $(env ...) or $(optenv ...) references
 */
export function detectArgumentsAndEnv(content: string): { hasArgs: boolean; hasEnv: boolean; argNames: string[]; envNames: string[] } {
    const argNames: string[] = [];
    const envNames: string[] = [];

    // Detect xacro:arg declarations
    const argPattern = /<xacro:arg\s+name="([^"]+)"/g;
    let match;
    while ((match = argPattern.exec(content)) !== null) {
        if (!argNames.includes(match[1])) {
            argNames.push(match[1]);
        }
    }

    // Detect $(arg name) usage
    const argUsagePattern = /\$\(arg\s+([a-zA-Z0-9_-]+)\)/g;
    while ((match = argUsagePattern.exec(content)) !== null) {
        if (!argNames.includes(match[1])) {
            argNames.push(match[1]);
        }
    }

    // Detect $(env name) usage
    const envPattern = /\$\(env\s+([a-zA-Z0-9_-]+)\)/g;
    while ((match = envPattern.exec(content)) !== null) {
        if (!envNames.includes(match[1])) {
            envNames.push(match[1]);
        }
    }

    // Detect $(optenv name default) usage
    const optenvPattern = /\$\(optenv\s+([a-zA-Z0-9_-]+)(?:\s+[^)]+)?\)/g;
    while ((match = optenvPattern.exec(content)) !== null) {
        if (!envNames.includes(match[1])) {
            envNames.push(match[1]);
        }
    }

    return {
        hasArgs: argNames.length > 0,
        hasEnv: envNames.length > 0,
        argNames,
        envNames
    };
}

/**
 * Prompts the user to create a xacro.json configuration file
 */
export async function promptCreateConfig(
    filePath: string,
    argNames: string[],
    envNames: string[],
    workspaceFolder?: vscode.WorkspaceFolder
): Promise<boolean> {
    const message = argNames.length > 0 && envNames.length > 0
        ? `The file contains ${argNames.length} argument(s) and ${envNames.length} environment variable(s). Would you like to create a configuration file to provide values?`
        : argNames.length > 0
        ? `The file contains ${argNames.length} xacro argument(s). Would you like to create a configuration file to provide values?`
        : `The file contains ${envNames.length} environment variable(s). Would you like to create a configuration file to provide values?`;

    const response = await vscode.window.showInformationMessage(
        message,
        'Yes',
        'No',
        'Don\'t ask again'
    );

    if (response !== 'Yes') {
        return false;
    }

    // Create or update the configuration
    let config = await loadXacroConfig(workspaceFolder);
    if (!config) {
        config = {
            version: '1.0.0'
        };
    }

    // Create relative path from workspace
    const workspaceRoot = workspaceFolder?.uri.fsPath || vscode.workspace.workspaceFolders?.[0]?.uri.fsPath || '';
    let relativeFilePath = path.relative(workspaceRoot, filePath).replace(/\\/g, '/');
    if (!relativeFilePath.startsWith('.')) {
        relativeFilePath = '${workspaceFolder}/' + relativeFilePath;
    }

    // Create file configuration
    const fileConfig: XacroFileConfig = {};
    
    if (argNames.length > 0) {
        fileConfig.args = {};
        argNames.forEach(argName => {
            fileConfig.args![argName] = '';  // Empty string as placeholder
        });
    }

    if (envNames.length > 0) {
        fileConfig.env = {};
        envNames.forEach(envName => {
            fileConfig.env![envName] = '';  // Empty string as placeholder
        });
    }

    config[relativeFilePath] = fileConfig;

    // Save configuration
    const saved = await saveXacroConfig(config, workspaceFolder);
    
    if (saved) {
        const configPath = getConfigFilePath(workspaceFolder);
        vscode.window.showInformationMessage(
            `Configuration file created at ${configPath}. Please fill in the values.`
        );
        
        // Open the config file for editing
        if (configPath) {
            const doc = await vscode.workspace.openTextDocument(configPath);
            await vscode.window.showTextDocument(doc);
        }
    }

    return saved;
}

/**
 * Adds a missing argument to the configuration file
 */
export async function addMissingArgument(
    filePath: string,
    argName: string,
    workspaceFolder?: vscode.WorkspaceFolder
): Promise<void> {
    let config = await loadXacroConfig(workspaceFolder);
    if (!config) {
        config = {
            version: '1.0.0'
        };
    }

    // Create relative path from workspace
    const workspaceRoot = workspaceFolder?.uri.fsPath || vscode.workspace.workspaceFolders?.[0]?.uri.fsPath || '';
    let relativeFilePath = path.relative(workspaceRoot, filePath).replace(/\\/g, '/');
    if (!relativeFilePath.startsWith('.')) {
        relativeFilePath = '${workspaceFolder}/' + relativeFilePath;
    }

    // Get or create file configuration
    let fileConfig = config[relativeFilePath] as XacroFileConfig;
    if (!fileConfig || typeof fileConfig === 'string') {
        fileConfig = {};
        config[relativeFilePath] = fileConfig;
    }

    // Add missing argument
    if (!fileConfig.args) {
        fileConfig.args = {};
    }
    fileConfig.args[argName] = '';

    // Save configuration
    await saveXacroConfig(config, workspaceFolder);

    const configPath = getConfigFilePath(workspaceFolder);
    vscode.window.showWarningMessage(
        `Missing argument '${argName}' was added to ${configPath}. Please provide a value.`,
        'Open Config'
    ).then(response => {
        if (response === 'Open Config' && configPath) {
            vscode.workspace.openTextDocument(configPath).then(doc => {
                vscode.window.showTextDocument(doc);
            });
        }
    });
}
