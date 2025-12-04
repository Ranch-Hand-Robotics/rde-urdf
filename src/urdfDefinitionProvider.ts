import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';

export class URDFDefinitionProvider implements vscode.DefinitionProvider {
    private outputChannel: vscode.OutputChannel;

    constructor(outputChannel: vscode.OutputChannel) {
        this.outputChannel = outputChannel;
    }

    async provideDefinition(
        document: vscode.TextDocument,
        position: vscode.Position,
        token: vscode.CancellationToken
    ): Promise<vscode.Definition | undefined> {
        const wordRange = document.getWordRangeAtPosition(position, /[\w-]+/);
        if (!wordRange) {
            return undefined;
        }

        const word = document.getText(wordRange);
        const line = document.lineAt(position.line).text;
        
        this.outputChannel.appendLine(`Looking for URDF definition of: ${word}`);

        // Determine what we're looking for based on context
        let searchType: 'link' | 'joint' | 'macro' | 'property' | 'file' | undefined;
        
        if (line.includes('link=') || line.includes('<child') || line.includes('<parent')) {
            searchType = 'link';
        } else if (line.includes('joint=') || line.includes('<joint')) {
            searchType = 'joint';
        } else if (line.includes('xacro:') && line.includes('name=')) {
            searchType = 'macro';
        } else if (line.includes('${') || line.includes('xacro:property')) {
            searchType = 'property';
        } else if (line.includes('filename=') || line.includes('<xacro:include')) {
            searchType = 'file';
        }

        // Handle file references (meshes, xacro includes, etc.)
        if (searchType === 'file' || line.includes('package://') || line.includes('filename=')) {
            return this.findFileDefinition(document, line, word);
        }

        // Search in current file first
        const currentFileDefinition = this.findDefinitionInFile(document, word, searchType);
        if (currentFileDefinition) {
            return currentFileDefinition;
        }

        // Search in included xacro files
        const includedFiles = await this.findIncludedFiles(document);
        for (const filePath of includedFiles) {
            try {
                const fileUri = vscode.Uri.file(filePath);
                const fileDoc = await vscode.workspace.openTextDocument(fileUri);
                const definition = this.findDefinitionInFile(fileDoc, word, searchType);
                if (definition) {
                    return definition;
                }
            } catch (error) {
                this.outputChannel.appendLine(`Could not open file: ${filePath}`);
            }
        }

        // Search in workspace files
        const workspaceDefinition = await this.searchWorkspace(word, searchType);
        if (workspaceDefinition) {
            return workspaceDefinition;
        }

        return undefined;
    }

    private findDefinitionInFile(
        document: vscode.TextDocument,
        symbolName: string,
        searchType?: string
    ): vscode.Location | undefined {
        const text = document.getText();
        let pattern: RegExp | undefined;

        switch (searchType) {
            case 'link':
                // Match <link name="symbolName"
                pattern = new RegExp(`<link\\s+name=["']${symbolName}["']`, 'm');
                break;
            case 'joint':
                // Match <joint name="symbolName"
                pattern = new RegExp(`<joint\\s+name=["']${symbolName}["']`, 'm');
                break;
            case 'macro':
                // Match <xacro:macro name="symbolName"
                pattern = new RegExp(`<xacro:macro\\s+name=["']${symbolName}["']`, 'm');
                break;
            case 'property':
                // Match <xacro:property name="symbolName"
                pattern = new RegExp(`<xacro:property\\s+name=["']${symbolName}["']`, 'm');
                break;
            default:
                // Try all patterns
                const patterns = [
                    new RegExp(`<link\\s+name=["']${symbolName}["']`, 'm'),
                    new RegExp(`<joint\\s+name=["']${symbolName}["']`, 'm'),
                    new RegExp(`<xacro:macro\\s+name=["']${symbolName}["']`, 'm'),
                    new RegExp(`<xacro:property\\s+name=["']${symbolName}["']`, 'm')
                ];
                
                for (const p of patterns) {
                    const match = text.match(p);
                    if (match && match.index !== undefined) {
                        const position = document.positionAt(match.index);
                        return new vscode.Location(document.uri, position);
                    }
                }
                return undefined;
        }

        const match = text.match(pattern);
        if (match && match.index !== undefined) {
            const position = document.positionAt(match.index);
            return new vscode.Location(document.uri, position);
        }

        return undefined;
    }

    private async findFileDefinition(
        document: vscode.TextDocument,
        line: string,
        word: string
    ): Promise<vscode.Location | undefined> {
        // Extract file path from various formats
        let filePath: string | undefined;
        
        // Match package://package_name/path/to/file
        const packageMatch = line.match(/package:\/\/([\w-]+)\/([\w\/.-]+)/);
        if (packageMatch) {
            const packageName = packageMatch[1];
            const relativePath = packageMatch[2];
            filePath = await this.resolvePackagePath(packageName, relativePath);
        }
        
        // Match filename="path/to/file"
        const filenameMatch = line.match(/filename=["']([\w\/.-]+)["']/);
        if (filenameMatch && !packageMatch) {
            const relativePath = filenameMatch[1];
            const documentDir = path.dirname(document.uri.fsPath);
            filePath = path.resolve(documentDir, relativePath);
        }

        // Match <xacro:include filename="..."
        const includeMatch = line.match(/<xacro:include\s+filename=["']([\w\/.-]+)["']/);
        if (includeMatch && !filePath) {
            const relativePath = includeMatch[1];
            const documentDir = path.dirname(document.uri.fsPath);
            filePath = path.resolve(documentDir, relativePath);
        }

        if (filePath && fs.existsSync(filePath)) {
            return new vscode.Location(vscode.Uri.file(filePath), new vscode.Position(0, 0));
        }

        return undefined;
    }

    private async resolvePackagePath(packageName: string, relativePath: string): Promise<string | undefined> {
        // Search for package.xml in workspace
        const packageFiles = await vscode.workspace.findFiles('**/package.xml', '**/node_modules/**', 100);
        
        for (const packageFile of packageFiles) {
            try {
                const content = await vscode.workspace.fs.readFile(packageFile);
                const text = Buffer.from(content).toString('utf8');
                
                // Check if this is the right package
                const nameMatch = text.match(/<name>([\w-]+)<\/name>/);
                if (nameMatch && nameMatch[1] === packageName) {
                    const packageDir = path.dirname(packageFile.fsPath);
                    const fullPath = path.join(packageDir, relativePath);
                    
                    if (fs.existsSync(fullPath)) {
                        return fullPath;
                    }
                }
            } catch (error) {
                // Continue searching
            }
        }
        
        return undefined;
    }

    private async findIncludedFiles(document: vscode.TextDocument): Promise<string[]> {
        const text = document.getText();
        const files: string[] = [];
        const documentDir = path.dirname(document.uri.fsPath);
        
        // Match <xacro:include filename="..." />
        const includePattern = /<xacro:include\s+filename=["']([\w\/.-]+)["']/g;
        let match;
        
        while ((match = includePattern.exec(text)) !== null) {
            const includedFile = match[1];
            
            // Try relative to current file
            let fullPath = path.resolve(documentDir, includedFile);
            if (fs.existsSync(fullPath)) {
                files.push(fullPath);
                continue;
            }
            
            // Try resolving as package path
            const packageMatch = includedFile.match(/^([\w-]+)\/(.*)/);
            if (packageMatch) {
                const resolved = await this.resolvePackagePath(packageMatch[1], packageMatch[2]);
                if (resolved) {
                    files.push(resolved);
                }
            }
        }
        
        return files;
    }

    private async searchWorkspace(
        symbolName: string,
        searchType?: string
    ): Promise<vscode.Location | undefined> {
        // Search for the symbol in all URDF/Xacro files in the workspace
        const files = await vscode.workspace.findFiles('**/*.{urdf,xacro}', '**/node_modules/**', 100);
        
        for (const fileUri of files) {
            try {
                const document = await vscode.workspace.openTextDocument(fileUri);
                const definition = this.findDefinitionInFile(document, symbolName, searchType);
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
