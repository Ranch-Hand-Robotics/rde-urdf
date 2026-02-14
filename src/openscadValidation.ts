import * as vscode from 'vscode';
import { validateOpenSCAD } from './openscad';

/**
 * OpenSCAD Validation Provider
 * 
 * This module provides real-time validation for OpenSCAD files including:
 * - Syntax validation via OpenSCAD compiler
 * - Runtime error detection
 * - Warning messages from compilation
 */

export class OpenSCADValidationProvider {
    private diagnosticCollection: vscode.DiagnosticCollection;
    private validationTimeout: NodeJS.Timeout | null = null;
    private readonly VALIDATION_DELAY = 500; // ms - debounce rapid typing

    constructor() {
        this.diagnosticCollection = vscode.languages.createDiagnosticCollection('openscad');
    }

    /**
     * Validates an OpenSCAD document and updates diagnostics
     */
    public async validateDocument(document: vscode.TextDocument): Promise<void> {
        // Only validate OpenSCAD files
        if (!this.shouldValidate(document)) {
            return;
        }

        // Clear any pending validation
        if (this.validationTimeout) {
            clearTimeout(this.validationTimeout);
        }

        // Debounce validation to avoid excessive compilation on rapid typing
        this.validationTimeout = setTimeout(async () => {
            await this.performValidation(document);
        }, this.VALIDATION_DELAY);
    }

    /**
     * Performs the actual validation
     */
    private async performValidation(document: vscode.TextDocument): Promise<void> {
        const diagnostics: vscode.Diagnostic[] = [];
        const text = document.getText();
        
        try {
            // Get workspace root for library resolution
            const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
            
            // Validate using OpenSCAD compiler
            const result = await validateOpenSCAD(
                document.uri.fsPath,
                text,
                workspaceRoot
            );

            // Process errors
            if (!result.valid && result.errors.length > 0) {
                result.errors.forEach(error => {
                    const diagnostic = this.parseErrorMessage(error, document);
                    if (diagnostic) {
                        diagnostics.push(diagnostic);
                    }
                });
            }

            // Process warnings
            if (result.warnings.length > 0) {
                result.warnings.forEach(warning => {
                    const diagnostic = this.parseWarningMessage(warning, document);
                    if (diagnostic) {
                        diagnostics.push(diagnostic);
                    }
                });
            }
        } catch (error) {
            // If validation completely fails, show a general error
            const diagnostic = new vscode.Diagnostic(
                new vscode.Range(0, 0, 0, 1),
                `OpenSCAD validation failed: ${error instanceof Error ? error.message : String(error)}`,
                vscode.DiagnosticSeverity.Error
            );
            diagnostic.source = 'openscad';
            diagnostics.push(diagnostic);
        }

        // Update diagnostics
        this.diagnosticCollection.set(document.uri, diagnostics);
    }

    /**
     * Check if the document should be validated
     */
    private shouldValidate(document: vscode.TextDocument): boolean {
        const fileName = document.fileName.toLowerCase();
        return fileName.endsWith('.scad');
    }

    /**
     * Parse an error message and create a diagnostic
     * OpenSCAD error format: "ERROR: Parser error: ... in file ..., line X"
     * or: "ERROR: ... in file ..., line X"
     */
    private parseErrorMessage(error: string, document: vscode.TextDocument): vscode.Diagnostic | null {
        // Skip empty errors
        if (!error || !error.trim()) {
            return null;
        }

        // Try to extract line number from error message
        // Common patterns:
        // "ERROR: ... line 5"
        // "line 5:"
        // "in file input.scad, line 5"
        const lineMatch = error.match(/line[:\s]+(\d+)/i);
        const line = lineMatch ? Math.max(0, parseInt(lineMatch[1], 10) - 1) : 0;

        // Try to extract column number if available
        const colMatch = error.match(/column[:\s]+(\d+)/i);
        const column = colMatch ? Math.max(0, parseInt(colMatch[1], 10) - 1) : 0;

        // Get the line text to determine range
        const lineText = document.lineAt(Math.min(line, document.lineCount - 1)).text;
        const endColumn = column > 0 ? Math.min(column + 10, lineText.length) : lineText.length;

        const range = new vscode.Range(
            new vscode.Position(line, column),
            new vscode.Position(line, endColumn)
        );

        // Clean up error message - remove file path and line number references
        let message = error
            .replace(/ERROR:\s*/i, '')
            .replace(/in file [^,]+,?\s*/i, '')
            .replace(/line \d+[:\s]*/i, '')
            .trim();

        // If message is empty after cleanup, use original
        if (!message) {
            message = error;
        }

        const diagnostic = new vscode.Diagnostic(
            range,
            message,
            vscode.DiagnosticSeverity.Error
        );
        diagnostic.source = 'openscad';
        
        return diagnostic;
    }

    /**
     * Parse a warning message and create a diagnostic
     */
    private parseWarningMessage(warning: string, document: vscode.TextDocument): vscode.Diagnostic | null {
        // Skip empty warnings
        if (!warning || !warning.trim()) {
            return null;
        }

        // Try to extract line number from warning message
        const lineMatch = warning.match(/line[:\s]+(\d+)/i);
        const line = lineMatch ? Math.max(0, parseInt(lineMatch[1], 10) - 1) : 0;

        // Try to extract column number if available
        const colMatch = warning.match(/column[:\s]+(\d+)/i);
        const column = colMatch ? Math.max(0, parseInt(colMatch[1], 10) - 1) : 0;

        // Get the line text to determine range
        const lineText = document.lineAt(Math.min(line, document.lineCount - 1)).text;
        const endColumn = column > 0 ? Math.min(column + 10, lineText.length) : lineText.length;

        const range = new vscode.Range(
            new vscode.Position(line, column),
            new vscode.Position(line, endColumn)
        );

        // Clean up warning message
        let message = warning
            .replace(/WARNING:\s*/i, '')
            .replace(/in file [^,]+,?\s*/i, '')
            .replace(/line \d+[:\s]*/i, '')
            .trim();

        if (!message) {
            message = warning;
        }

        const diagnostic = new vscode.Diagnostic(
            range,
            message,
            vscode.DiagnosticSeverity.Warning
        );
        diagnostic.source = 'openscad';
        
        return diagnostic;
    }

    /**
     * Clear diagnostics for a document
     */
    public clearDiagnostics(document: vscode.TextDocument): void {
        this.diagnosticCollection.delete(document.uri);
    }

    /**
     * Clear all diagnostics
     */
    public clearAll(): void {
        this.diagnosticCollection.clear();
    }

    /**
     * Dispose of the diagnostic collection and clear timeouts
     */
    public dispose(): void {
        if (this.validationTimeout) {
            clearTimeout(this.validationTimeout);
        }
        this.diagnosticCollection.dispose();
    }
}
