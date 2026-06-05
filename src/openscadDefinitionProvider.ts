import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';

export class OpenSCADDefinitionProvider implements vscode.DefinitionProvider {
    private outputChannel: vscode.OutputChannel;

    constructor(outputChannel: vscode.OutputChannel) {
        this.outputChannel = outputChannel;
    }

    async provideDefinition(
        document: vscode.TextDocument,
        position: vscode.Position,
        _token: vscode.CancellationToken
    ): Promise<vscode.Definition | undefined> {
        const wordRange = document.getWordRangeAtPosition(position);
        if (!wordRange) {
            return undefined;
        }

        const word = document.getText(wordRange);
        this.outputChannel.appendLine(`Looking for definition of: ${word}`);

        const currentFileDefinition = this.findDefinitionInFile(document, word);
        if (currentFileDefinition) {
            return currentFileDefinition;
        }

        const includedFiles = await this.findIncludedFiles(document);
        for (const filePath of includedFiles) {
            try {
                const fileUri = vscode.Uri.file(filePath);
                const fileDoc = await vscode.workspace.openTextDocument(fileUri);
                const definition = this.findDefinitionInFile(fileDoc, word);
                if (definition) {
                    return definition;
                }
            } catch {
                this.outputChannel.appendLine(`Could not open file: ${filePath}`);
            }
        }

        const workspaceDefinition = await this.searchWorkspace(word);
        if (workspaceDefinition) {
            return workspaceDefinition;
        }

        return undefined;
    }

    private findDefinitionInFile(document: vscode.TextDocument, symbolName: string): vscode.Location | undefined {
        const text = document.getText();
        const escapedSymbol = symbolName.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');

        const modulePattern = new RegExp(`\\bmodule\\s+${escapedSymbol}\\s*\\(`);
        const functionPattern = new RegExp(`\\bfunction\\s+${escapedSymbol}\\s*\\(`);

        let match = text.match(modulePattern);
        if (!match) {
            match = text.match(functionPattern);
        }

        if (match && match.index !== undefined) {
            const position = document.positionAt(match.index);
            return new vscode.Location(document.uri, position);
        }

        return undefined;
    }

    // Helper method to extract documentation for hover
    public getSymbolDocumentation(document: vscode.TextDocument, symbolName: string): string | undefined {
        const text = document.getText();
        const escapedSymbol = symbolName.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');

        const modulePattern = new RegExp(`\\bmodule\\s+${escapedSymbol}\\s*\\([^)]*\\)`);
        const functionPattern = new RegExp(`\\bfunction\\s+${escapedSymbol}\\s*\\([^)]*\\)\\s*=`);

        let match = text.match(modulePattern);
        if (!match) {
            match = text.match(functionPattern);
        }

        if (match && match.index !== undefined) {
            const signature = match[0];
            const textBeforeMatch = text.substring(0, match.index);
            const lines = textBeforeMatch.split('\n');
            const commentLines: string[] = [];
            let foundNonEmpty = false;

            for (let i = lines.length - 1; i >= 0; i--) {
                const line = lines[i].trim();
                const lineContent = line.startsWith('//') ? line.substring(2).trim() : '';

                if (line.startsWith('//')) {
                    if (
                        lineContent.match(/^[=\-_*]{3,}$/) ||
                        (lineContent.toUpperCase() === lineContent && lineContent.length > 10) ||
                        lineContent === ''
                    ) {
                        if (foundNonEmpty) {
                            break;
                        }
                        continue;
                    }
                    commentLines.unshift(lineContent);
                    foundNonEmpty = true;
                } else if (line === '') {
                    if (foundNonEmpty && commentLines.length > 0) {
                        const lastLine = commentLines[commentLines.length - 1];
                        if (lastLine === '') {
                            break;
                        }
                    }
                    continue;
                } else {
                    break;
                }
            }

            while (commentLines.length > 0 && commentLines[0] === '') {
                commentLines.shift();
            }
            while (commentLines.length > 0 && commentLines[commentLines.length - 1] === '') {
                commentLines.pop();
            }

            let doc = `\`\`\`openscad\n${signature}\n\`\`\``;
            if (commentLines.length > 0) {
                doc += '\n\n' + commentLines.join('\n');
            }
            return doc;
        }

        return undefined;
    }

    private async findIncludedFiles(document: vscode.TextDocument): Promise<string[]> {
        const text = document.getText();
        const files: string[] = [];
        const documentDir = path.dirname(document.uri.fsPath);

        const includePattern = /(?:include|use)\s*<([^>]+)>/g;
        let match;

        while ((match = includePattern.exec(text)) !== null) {
            const includedFile = match[1];

            let fullPath = path.resolve(documentDir, includedFile);
            if (fs.existsSync(fullPath)) {
                files.push(fullPath);
                continue;
            }

            const libraryPaths = await this.getLibraryPaths();
            for (const libPath of libraryPaths) {
                fullPath = path.resolve(libPath, includedFile);
                if (fs.existsSync(fullPath)) {
                    files.push(fullPath);
                    break;
                }
            }
        }

        return files;
    }

    private async getLibraryPaths(): Promise<string[]> {
        const paths: string[] = [];

        const workspaceFolders = vscode.workspace.workspaceFolders;
        if (workspaceFolders) {
            for (const folder of workspaceFolders) {
                paths.push(folder.uri.fsPath);
            }
        }

        const config = vscode.workspace.getConfiguration('urdf-editor');
        const configuredPaths = config.get<string[]>('OpenSCADLibraryPaths', []);

        for (const configPath of configuredPaths) {
            let resolvedPath = configPath;
            if (workspaceFolders && configPath.includes('${workspaceFolder}')) {
                resolvedPath = configPath.replace('${workspaceFolder}', workspaceFolders[0].uri.fsPath);
            }
            if (fs.existsSync(resolvedPath)) {
                paths.push(resolvedPath);
            }
        }

        return paths;
    }

    private async searchWorkspace(symbolName: string): Promise<vscode.Location | undefined> {
        const files = await vscode.workspace.findFiles('**/*.scad', '**/node_modules/**', 100);

        for (const fileUri of files) {
            try {
                const document = await vscode.workspace.openTextDocument(fileUri);
                const definition = this.findDefinitionInFile(document, symbolName);
                if (definition) {
                    return definition;
                }
            } catch {
                // Continue searching other files
            }
        }

        return undefined;
    }
}
