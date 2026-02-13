import * as vscode from 'vscode';
import { DOMParser } from 'xmldom';

/**
 * URDF/Xacro Validation Provider
 * 
 * This module provides validation for URDF and Xacro files including:
 * - XML syntax validation
 * - URDF structure validation (required elements, attributes)
 * - Character set validation (invalid characters)
 * - Reference validation (link/joint references)
 */

export class URDFXacroValidationProvider {
    private diagnosticCollection: vscode.DiagnosticCollection;

    constructor() {
        this.diagnosticCollection = vscode.languages.createDiagnosticCollection('urdf-xacro');
    }

    /**
     * Validates a URDF/Xacro document and updates diagnostics
     */
    public async validateDocument(document: vscode.TextDocument): Promise<void> {
        // Only validate URDF and Xacro files
        if (!this.shouldValidate(document)) {
            return;
        }

        const diagnostics: vscode.Diagnostic[] = [];
        const text = document.getText();

        // Check for invalid characters (non-UTF-8 or control characters)
        this.validateCharacterSet(text, document, diagnostics);

        // Parse and validate XML structure
        this.validateXMLStructure(text, document, diagnostics);

        // Validate URDF-specific requirements
        this.validateURDFStructure(text, document, diagnostics);

        // Update diagnostics
        this.diagnosticCollection.set(document.uri, diagnostics);
    }

    /**
     * Check if the document should be validated
     */
    private shouldValidate(document: vscode.TextDocument): boolean {
        const fileName = document.fileName.toLowerCase();
        return fileName.endsWith('.urdf') || fileName.endsWith('.xacro');
    }

    /**
     * Validates character set - checks for invalid or problematic characters
     */
    private validateCharacterSet(text: string, document: vscode.TextDocument, diagnostics: vscode.Diagnostic[]): void {
        const lines = text.split('\n');
        
        for (let i = 0; i < lines.length; i++) {
            const line = lines[i];
            
            // Check for invalid control characters (excluding valid ones like tab, newline)
            for (let j = 0; j < line.length; j++) {
                const charCode = line.charCodeAt(j);
                
                // Control characters except tab (9), newline (10), carriage return (13)
                if (charCode < 32 && charCode !== 9 && charCode !== 10 && charCode !== 13) {
                    const range = new vscode.Range(
                        new vscode.Position(i, j),
                        new vscode.Position(i, j + 1)
                    );
                    const diagnostic = new vscode.Diagnostic(
                        range,
                        `Invalid control character (code ${charCode}) found. URDF/Xacro files must use valid UTF-8 characters.`,
                        vscode.DiagnosticSeverity.Error
                    );
                    diagnostic.source = 'urdf-validator';
                    diagnostics.push(diagnostic);
                }

                // Check for invalid UTF-8 sequences or problematic characters
                // BOM (Byte Order Mark) at positions other than start of file
                if (charCode === 0xFEFF && !(i === 0 && j === 0)) {
                    const range = new vscode.Range(
                        new vscode.Position(i, j),
                        new vscode.Position(i, j + 1)
                    );
                    const diagnostic = new vscode.Diagnostic(
                        range,
                        'Byte Order Mark (BOM) found in invalid position. Remove this character.',
                        vscode.DiagnosticSeverity.Warning
                    );
                    diagnostic.source = 'urdf-validator';
                    diagnostics.push(diagnostic);
                }
            }
        }
    }

    /**
     * Validates XML structure using DOM parser
     */
    private validateXMLStructure(text: string, document: vscode.TextDocument, diagnostics: vscode.Diagnostic[]): void {
        const errors: Array<{ line: number; column: number; message: string }> = [];
        
        const errorHandler = {
            warning: (msg: string) => {
                // Parse line/column from error message if available
                this.parseAndAddError(msg, errors, 'warning');
            },
            error: (msg: string) => {
                this.parseAndAddError(msg, errors, 'error');
            },
            fatalError: (msg: string) => {
                this.parseAndAddError(msg, errors, 'error');
            }
        };

        try {
            const parser = new DOMParser({
                errorHandler: errorHandler
            });
            parser.parseFromString(text, 'text/xml');
        } catch (e) {
            // If parsing completely fails, add a generic error
            const diagnostic = new vscode.Diagnostic(
                new vscode.Range(0, 0, 0, 0),
                `XML parsing error: ${e instanceof Error ? e.message : String(e)}`,
                vscode.DiagnosticSeverity.Error
            );
            diagnostic.source = 'urdf-validator';
            diagnostics.push(diagnostic);
        }

        // Convert collected errors to diagnostics
        errors.forEach(err => {
            const line = Math.max(0, err.line - 1); // Convert to 0-based
            const column = Math.max(0, err.column - 1);
            
            // Try to get the actual line length for range
            const lines = text.split('\n');
            const lineText = lines[line] || '';
            const endColumn = Math.min(column + 10, lineText.length);
            
            const range = new vscode.Range(
                new vscode.Position(line, column),
                new vscode.Position(line, endColumn)
            );
            
            const diagnostic = new vscode.Diagnostic(
                range,
                err.message,
                vscode.DiagnosticSeverity.Error
            );
            diagnostic.source = 'urdf-validator';
            diagnostics.push(diagnostic);
        });
    }

    /**
     * Parse error messages from XML parser to extract line/column information
     */
    private parseAndAddError(
        msg: string, 
        errors: Array<{ line: number; column: number; message: string }>,
        type: 'warning' | 'error'
    ): void {
        // XML errors often come in format like "@#[line:X,col:Y]message" or similar
        // Try to extract line and column numbers
        const lineMatch = msg.match(/line[:\s]+(\d+)/i);
        const colMatch = msg.match(/col(?:umn)?[:\s]+(\d+)/i);
        
        const line = lineMatch ? parseInt(lineMatch[1], 10) : 1;
        const column = colMatch ? parseInt(colMatch[1], 10) : 1;
        
        // Clean up the message
        let cleanMessage = msg.replace(/@#\[line:\d+,col:\d+\]/g, '').trim();
        cleanMessage = cleanMessage || `XML ${type}: ${msg}`;
        
        errors.push({ line, column, message: cleanMessage });
    }

    /**
     * Validates URDF-specific structure and requirements
     */
    private validateURDFStructure(text: string, document: vscode.TextDocument, diagnostics: vscode.Diagnostic[]): void {
        try {
            const parser = new DOMParser({
                errorHandler: {
                    warning: () => {},
                    error: () => {},
                    fatalError: () => {}
                }
            });
            const doc = parser.parseFromString(text, 'text/xml');
            
            // Check for root robot element
            const robotElements = doc.getElementsByTagName('robot');
            if (robotElements.length === 0) {
                const diagnostic = new vscode.Diagnostic(
                    new vscode.Range(0, 0, 0, 10),
                    'URDF file must have a root <robot> element',
                    vscode.DiagnosticSeverity.Error
                );
                diagnostic.source = 'urdf-validator';
                diagnostics.push(diagnostic);
                return; // Can't continue validation without robot element
            }

            const robot = robotElements[0];
            
            // Check for robot name attribute
            if (!robot.hasAttribute('name') && !robot.hasAttributeNS('http://www.ros.org/wiki/xacro', 'name')) {
                const diagnostic = new vscode.Diagnostic(
                    new vscode.Range(0, 0, 0, 10),
                    'Robot element must have a "name" attribute',
                    vscode.DiagnosticSeverity.Error
                );
                diagnostic.source = 'urdf-validator';
                diagnostics.push(diagnostic);
            }

            // Collect all link names for reference validation
            const linkNames = new Set<string>();
            const links = robot.getElementsByTagName('link');
            for (let i = 0; i < links.length; i++) {
                const link = links[i];
                const linkName = link.getAttribute('name');
                if (linkName) {
                    linkNames.add(linkName);
                } else {
                    // Find position of this link element in source
                    const diagnostic = new vscode.Diagnostic(
                        this.findElementPosition(text, 'link', i),
                        'Link element must have a "name" attribute',
                        vscode.DiagnosticSeverity.Error
                    );
                    diagnostic.source = 'urdf-validator';
                    diagnostics.push(diagnostic);
                }
            }

            // Validate joint references
            const joints = robot.getElementsByTagName('joint');
            for (let i = 0; i < joints.length; i++) {
                const joint = joints[i];
                const jointName = joint.getAttribute('name');
                
                if (!jointName) {
                    const diagnostic = new vscode.Diagnostic(
                        this.findElementPosition(text, 'joint', i),
                        'Joint element must have a "name" attribute',
                        vscode.DiagnosticSeverity.Error
                    );
                    diagnostic.source = 'urdf-validator';
                    diagnostics.push(diagnostic);
                }

                // Check joint type
                const jointType = joint.getAttribute('type');
                const validTypes = ['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar'];
                if (jointType && !validTypes.includes(jointType)) {
                    const diagnostic = new vscode.Diagnostic(
                        this.findElementPosition(text, 'joint', i),
                        `Invalid joint type "${jointType}". Valid types are: ${validTypes.join(', ')}`,
                        vscode.DiagnosticSeverity.Error
                    );
                    diagnostic.source = 'urdf-validator';
                    diagnostics.push(diagnostic);
                }

                // Validate parent/child link references
                const parents = joint.getElementsByTagName('parent');
                const children = joint.getElementsByTagName('child');
                
                if (parents.length > 0) {
                    const parentLink = parents[0].getAttribute('link');
                    if (parentLink && !linkNames.has(parentLink) && !this.isXacroReference(parentLink)) {
                        const diagnostic = new vscode.Diagnostic(
                            this.findElementPosition(text, 'parent', 0, joint),
                            `Parent link "${parentLink}" not found. Define this link or check for typos.`,
                            vscode.DiagnosticSeverity.Warning
                        );
                        diagnostic.source = 'urdf-validator';
                        diagnostics.push(diagnostic);
                    }
                }

                if (children.length > 0) {
                    const childLink = children[0].getAttribute('link');
                    if (childLink && !linkNames.has(childLink) && !this.isXacroReference(childLink)) {
                        const diagnostic = new vscode.Diagnostic(
                            this.findElementPosition(text, 'child', 0, joint),
                            `Child link "${childLink}" not found. Define this link or check for typos.`,
                            vscode.DiagnosticSeverity.Warning
                        );
                        diagnostic.source = 'urdf-validator';
                        diagnostics.push(diagnostic);
                    }
                }
            }

            // Validate geometry elements
            this.validateGeometry(doc, text, diagnostics);

        } catch (e) {
            // XML parsing error already handled in validateXMLStructure
        }
    }

    /**
     * Check if a reference might be a Xacro variable/expression
     */
    private isXacroReference(value: string): boolean {
        return value.includes('${') || value.includes('$(');
    }

    /**
     * Validates geometry elements (box, cylinder, sphere, mesh)
     */
    private validateGeometry(doc: Document, text: string, diagnostics: vscode.Diagnostic[]): void {
        // Validate box elements
        const boxes = doc.getElementsByTagName('box');
        for (let i = 0; i < boxes.length; i++) {
            const box = boxes[i];
            const size = box.getAttribute('size');
            if (!size && !this.hasXacroAttribute(box, 'size')) {
                const diagnostic = new vscode.Diagnostic(
                    this.findElementPosition(text, 'box', i),
                    'Box geometry must have a "size" attribute (e.g., size="1 1 1")',
                    vscode.DiagnosticSeverity.Error
                );
                diagnostic.source = 'urdf-validator';
                diagnostics.push(diagnostic);
            } else if (size && !this.isXacroReference(size)) {
                // Validate size format (should be 3 numbers)
                const parts = size.trim().split(/\s+/);
                if (parts.length !== 3 || !parts.every(p => !isNaN(parseFloat(p)))) {
                    const diagnostic = new vscode.Diagnostic(
                        this.findElementPosition(text, 'box', i),
                        'Box size must be three numbers (e.g., "1 1 1" for x y z dimensions)',
                        vscode.DiagnosticSeverity.Error
                    );
                    diagnostic.source = 'urdf-validator';
                    diagnostics.push(diagnostic);
                }
            }
        }

        // Validate cylinder elements
        const cylinders = doc.getElementsByTagName('cylinder');
        for (let i = 0; i < cylinders.length; i++) {
            const cylinder = cylinders[i];
            const radius = cylinder.getAttribute('radius');
            const length = cylinder.getAttribute('length');
            
            if (!radius && !this.hasXacroAttribute(cylinder, 'radius')) {
                const diagnostic = new vscode.Diagnostic(
                    this.findElementPosition(text, 'cylinder', i),
                    'Cylinder geometry must have a "radius" attribute',
                    vscode.DiagnosticSeverity.Error
                );
                diagnostic.source = 'urdf-validator';
                diagnostics.push(diagnostic);
            }
            
            if (!length && !this.hasXacroAttribute(cylinder, 'length')) {
                const diagnostic = new vscode.Diagnostic(
                    this.findElementPosition(text, 'cylinder', i),
                    'Cylinder geometry must have a "length" attribute',
                    vscode.DiagnosticSeverity.Error
                );
                diagnostic.source = 'urdf-validator';
                diagnostics.push(diagnostic);
            }
        }

        // Validate sphere elements
        const spheres = doc.getElementsByTagName('sphere');
        for (let i = 0; i < spheres.length; i++) {
            const sphere = spheres[i];
            const radius = sphere.getAttribute('radius');
            
            if (!radius && !this.hasXacroAttribute(sphere, 'radius')) {
                const diagnostic = new vscode.Diagnostic(
                    this.findElementPosition(text, 'sphere', i),
                    'Sphere geometry must have a "radius" attribute',
                    vscode.DiagnosticSeverity.Error
                );
                diagnostic.source = 'urdf-validator';
                diagnostics.push(diagnostic);
            }
        }

        // Validate mesh elements
        const meshes = doc.getElementsByTagName('mesh');
        for (let i = 0; i < meshes.length; i++) {
            const mesh = meshes[i];
            const filename = mesh.getAttribute('filename');
            
            if (!filename && !this.hasXacroAttribute(mesh, 'filename')) {
                const diagnostic = new vscode.Diagnostic(
                    this.findElementPosition(text, 'mesh', i),
                    'Mesh geometry must have a "filename" attribute',
                    vscode.DiagnosticSeverity.Error
                );
                diagnostic.source = 'urdf-validator';
                diagnostics.push(diagnostic);
            }
        }
    }

    /**
     * Check if element has a Xacro-namespaced attribute
     */
    private hasXacroAttribute(element: Element, attrName: string): boolean {
        // Check for xacro:attribute_name
        const attrs = element.attributes;
        for (let i = 0; i < attrs.length; i++) {
            const attr = attrs[i];
            if (attr.name.includes('xacro:') && attr.name.includes(attrName)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Find the position of an element in the source text
     * This is a simple implementation that searches for the nth occurrence
     */
    private findElementPosition(
        text: string, 
        tagName: string, 
        index: number,
        parentElement?: Element
    ): vscode.Range {
        const lines = text.split('\n');
        const pattern = new RegExp(`<${tagName}[\\s>]`, 'gi');
        let count = 0;
        
        for (let i = 0; i < lines.length; i++) {
            const line = lines[i];
            const matches = line.matchAll(pattern);
            
            for (const match of matches) {
                if (count === index) {
                    const startCol = match.index || 0;
                    const endCol = Math.min(startCol + tagName.length + 2, line.length);
                    return new vscode.Range(
                        new vscode.Position(i, startCol),
                        new vscode.Position(i, endCol)
                    );
                }
                count++;
            }
        }
        
        // Fallback to start of document
        return new vscode.Range(0, 0, 0, 10);
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
     * Dispose of the diagnostic collection
     */
    public dispose(): void {
        this.diagnosticCollection.dispose();
    }
}
