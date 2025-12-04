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
        token: vscode.CancellationToken
    ): Promise<vscode.Definition | undefined> {
        const wordRange = document.getWordRangeAtPosition(position);
        if (!wordRange) {
            return undefined;
        }

        const word = document.getText(wordRange);
        this.outputChannel.appendLine(`Looking for definition of: ${word}`);

        // Search in current file first
        const currentFileDefinition = this.findDefinitionInFile(document, word);
        if (currentFileDefinition) {
            return currentFileDefinition;
        }

        // Search in included/used files
        const includedFiles = await this.findIncludedFiles(document);
        for (const filePath of includedFiles) {
            try {
                const fileUri = vscode.Uri.file(filePath);
                const fileDoc = await vscode.workspace.openTextDocument(fileUri);
                const definition = this.findDefinitionInFile(fileDoc, word);
                if (definition) {
                    return definition;
                }
            } catch (error) {
                // File not found or can't be opened
                this.outputChannel.appendLine(`Could not open file: ${filePath}`);
            }
        }

        // Search in workspace files
        const workspaceDefinition = await this.searchWorkspace(word);
        if (workspaceDefinition) {
            return workspaceDefinition;
        }

        return undefined;
    }

    private findDefinitionInFile(
        document: vscode.TextDocument,
        symbolName: string
    ): vscode.Location | undefined {
        const text = document.getText();
        
        // Escape special regex characters in symbol name
        const escapedSymbol = symbolName.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
        
        // Match module definitions: module name(...) { or module name() {
        const modulePattern = new RegExp(`\\bmodule\\s+${escapedSymbol}\\s*\\(`);
        // Match function definitions: function name(...) = or function name() =
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
        
        // Match module/function with full signature
        const modulePattern = new RegExp(`\\bmodule\\s+${escapedSymbol}\\s*\\([^)]*\\)`);
        const functionPattern = new RegExp(`\\bfunction\\s+${escapedSymbol}\\s*\\([^)]*\\)\\s*=`);
        
        let match = text.match(modulePattern);
        let type = 'module';
        if (!match) {
            match = text.match(functionPattern);
            type = 'function';
        }
        
        if (match && match.index !== undefined) {
            const signature = match[0];
            
            // Find preceding comment block
            const textBeforeMatch = text.substring(0, match.index);
            const lines = textBeforeMatch.split('\n');
            const commentLines: string[] = [];
            let foundNonEmpty = false;
            
            for (let i = lines.length - 1; i >= 0; i--) {
                const line = lines[i].trim();
                const lineContent = line.startsWith('//') ? line.substring(2).trim() : '';
                
                if (line.startsWith('//')) {
                    // Skip section headers and decorative comments
                    if (lineContent.match(/^[=\-_*]{3,}$/) || // Lines of ===, ---, ___, ***
                        lineContent.toUpperCase() === lineContent && lineContent.length > 10 || // ALL CAPS headers
                        lineContent === '') { // Empty comment lines at the start
                        if (foundNonEmpty) {
                            break; // Stop if we've already found real content
                        }
                        continue; // Skip these decorative lines
                    }
                    
                    commentLines.unshift(lineContent);
                    foundNonEmpty = true;
                } else if (line === '') {
                    // Allow one empty line, but stop if we have content and hit another
                    if (foundNonEmpty && commentLines.length > 0) {
                        const lastLine = commentLines[commentLines.length - 1];
                        // Check if the last line we added seems to be the start of a block
                        if (lastLine === '') {
                            break;
                        }
                    }
                    continue;
                } else {
                    // Non-comment, non-empty line - stop
                    break;
                }
            }
            
            // Clean up: remove leading/trailing empty lines
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
        
        // Match include <file> or use <file>
        const includePattern = /(?:include|use)\s*<([^>]+)>/g;
        let match;
        
        while ((match = includePattern.exec(text)) !== null) {
            const includedFile = match[1];
            
            // Try relative to current file
            let fullPath = path.resolve(documentDir, includedFile);
            if (fs.existsSync(fullPath)) {
                files.push(fullPath);
                continue;
            }
            
            // Try in library paths
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
        
        // Add workspace root
        const workspaceFolders = vscode.workspace.workspaceFolders;
        if (workspaceFolders) {
            for (const folder of workspaceFolders) {
                paths.push(folder.uri.fsPath);
            }
        }
        
        // Add configured library paths
        const config = vscode.workspace.getConfiguration('urdf-editor');
        const configuredPaths = config.get<string[]>('OpenSCADLibraryPaths', []);
        
        for (const configPath of configuredPaths) {
            // Resolve ${workspaceFolder} variable
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
        // Search for the symbol in all .scad files in the workspace
        const files = await vscode.workspace.findFiles('**/*.scad', '**/node_modules/**', 100);
        
        for (const fileUri of files) {
            try {
                const document = await vscode.workspace.openTextDocument(fileUri);
                const definition = this.findDefinitionInFile(document, symbolName);
                if (definition) {
                    return definition;
                }
            } catch (error) {
                // Continue searching other files
            }
        }
        
        return undefined;
    }
}
