// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// URDF and Xacro IntelliSense completion provider

import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';
import { tracing } from './extension';

// URDF elements with full signatures
const urdfElements = [
    { name: 'robot', snippet: 'robot name="$1">\n\t$0\n</robot>', signature: '<robot name="...">', description: 'Root element of a URDF file', attributes: 'name: string (required) - Unique name of the robot' },
    { name: 'link', snippet: 'link name="$1">\n\t$0\n</link>', signature: '<link name="...">', description: 'Defines a rigid body with inertia, visual, and collision properties', attributes: 'name: string (required) - Unique name of the link' },
    { name: 'joint', snippet: 'joint name="$1" type="$2">\n\t<parent link="$3"/>\n\t<child link="$4"/>\n\t$0\n</joint>', signature: '<joint name="..." type="revolute|continuous|prismatic|fixed|floating|planar">', description: 'Connects two links together', attributes: 'name: string (required) - Unique name of the joint\ntype: enum (required) - Type of joint motion' },
    { name: 'visual', snippet: 'visual>\n\t<geometry>\n\t\t$0\n\t</geometry>\n</visual>', signature: '<visual name="...">', description: 'Visual representation of a link', attributes: 'name: string (optional) - Name for multiple visual elements' },
    { name: 'collision', snippet: 'collision>\n\t<geometry>\n\t\t$0\n\t</geometry>\n</collision>', signature: '<collision name="...">', description: 'Collision shape for physics simulation', attributes: 'name: string (optional) - Name for multiple collision elements' },
    { name: 'inertial', snippet: 'inertial>\n\t<origin xyz="0 0 0" rpy="0 0 0"/>\n\t<mass value="$1"/>\n\t<inertia ixx="$2" ixy="0" ixz="0" iyy="$3" iyz="0" izz="$4"/>\n</inertial>', signature: '<inertial>', description: 'Inertial properties of a link', attributes: 'Contains: <origin>, <mass>, <inertia> elements' },
    { name: 'geometry', snippet: 'geometry>\n\t$0\n</geometry>', signature: '<geometry>', description: 'Geometric shape definition', attributes: 'Contains: <box>, <cylinder>, <sphere>, or <mesh> element' },
    { name: 'origin', snippet: 'origin xyz="$1" rpy="$2"/>', signature: '<origin xyz="x y z" rpy="roll pitch yaw"/>', description: 'Position and orientation (xyz in meters, rpy in radians)', attributes: 'xyz: vector3 (default: "0 0 0") - Position in meters\nrpy: vector3 (default: "0 0 0") - Orientation in radians' },
    { name: 'parent', snippet: 'parent link="$1"/>', signature: '<parent link="..."/>', description: 'Parent link in a joint', attributes: 'link: string (required) - Name of parent link' },
    { name: 'child', snippet: 'child link="$1"/>', signature: '<child link="..."/>', description: 'Child link in a joint', attributes: 'link: string (required) - Name of child link' },
    { name: 'axis', snippet: 'axis xyz="$1"/>', signature: '<axis xyz="x y z"/>', description: 'Axis of rotation or translation for a joint', attributes: 'xyz: vector3 (default: "1 0 0") - Axis direction (normalized)' },
    { name: 'limit', snippet: 'limit lower="$1" upper="$2" effort="$3" velocity="$4"/>', signature: '<limit lower="..." upper="..." effort="..." velocity="..."/>', description: 'Joint limits', attributes: 'lower: float (required) - Lower limit (m or rad)\nupper: float (required) - Upper limit (m or rad)\neffort: float (required) - Max force/torque (N or Nm)\nvelocity: float (required) - Max velocity (m/s or rad/s)' },
    { name: 'dynamics', snippet: 'dynamics damping="$1" friction="$2"/>', signature: '<dynamics damping="..." friction="..."/>', description: 'Physical properties for joint motion', attributes: 'damping: float (default: 0) - Damping coefficient\nfriction: float (default: 0) - Friction coefficient' },
    { name: 'calibration', snippet: 'calibration rising="$1" falling="$2"/>', signature: '<calibration rising="..." falling="..."/>', description: 'Joint calibration reference positions', attributes: 'rising: float (optional) - Rising edge position\nfalling: float (optional) - Falling edge position' },
    { name: 'safety_controller', snippet: 'safety_controller soft_lower_limit="$1" soft_upper_limit="$2" k_position="$3" k_velocity="$4"/>', signature: '<safety_controller soft_lower_limit="..." soft_upper_limit="..." k_position="..." k_velocity="..."/>', description: 'Safety limits and gains', attributes: 'soft_lower_limit: float - Soft lower limit\nsoft_upper_limit: float - Soft upper limit\nk_position: float - Position gain\nk_velocity: float (required) - Velocity gain' },
    { name: 'mimic', snippet: 'mimic joint="$1" multiplier="$2" offset="$3"/>', signature: '<mimic joint="..." multiplier="..." offset="..."/>', description: 'Makes this joint mimic another joint', attributes: 'joint: string (required) - Name of joint to mimic\nmultiplier: float (default: 1.0) - Position multiplier\noffset: float (default: 0.0) - Position offset' },
    { name: 'mass', snippet: 'mass value="$1"/>', signature: '<mass value="..."/>', description: 'Mass of the link', attributes: 'value: float (required) - Mass in kilograms' },
    { name: 'inertia', snippet: 'inertia ixx="$1" ixy="0" ixz="0" iyy="$2" iyz="0" izz="$3"/>', signature: '<inertia ixx="..." ixy="..." ixz="..." iyy="..." iyz="..." izz="..."/>', description: 'Inertia tensor of the link', attributes: 'ixx, iyy, izz: float (required) - Diagonal moments of inertia (kg·m²)\nixy, ixz, iyz: float (required) - Off-diagonal products of inertia (kg·m²)' },
];

// Geometry types
const geometryTypes = [
    { name: 'box', snippet: 'box size="$1"/>', signature: '<box size="x y z"/>', description: 'Rectangular box', attributes: 'size: vector3 (required) - Dimensions in meters (x y z)' },
    { name: 'cylinder', snippet: 'cylinder radius="$1" length="$2"/>', signature: '<cylinder radius="..." length="..."/>', description: 'Cylinder shape', attributes: 'radius: float (required) - Radius in meters\nlength: float (required) - Length along Z-axis in meters' },
    { name: 'sphere', snippet: 'sphere radius="$1"/>', signature: '<sphere radius="..."/>', description: 'Sphere shape', attributes: 'radius: float (required) - Radius in meters' },
    { name: 'mesh', snippet: 'mesh filename="$1" scale="1 1 1"/>', signature: '<mesh filename="..." scale="x y z"/>', description: 'Load 3D mesh file', attributes: 'filename: string (required) - Path to mesh (STL, DAE, OBJ, GLB, GLTF)\nscale: vector3 (default: "1 1 1") - Scaling factors' },
];

// Material elements
const materialElements = [
    { name: 'material', snippet: 'material name="$1">\n\t<color rgba="$2"/>\n</material>', signature: '<material name="...">', description: 'Material definition with color', attributes: 'name: string (required) - Material name for reuse' },
    { name: 'color', snippet: 'color rgba="$1"/>', signature: '<color rgba="r g b a"/>', description: 'Color in RGBA format', attributes: 'rgba: vector4 (required) - Red, green, blue, alpha (0.0 to 1.0)' },
    { name: 'texture', snippet: 'texture filename="$1"/>', signature: '<texture filename="..."/>', description: 'Texture image file', attributes: 'filename: string (required) - Path to texture image' },
];

// Joint types
const jointTypes = [
    'revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar'
];

// Xacro-specific elements
const xacroElements = [
    { name: 'xacro:property', snippet: 'xacro:property name="$1" value="$2"/>', signature: '<xacro:property name="..." value="..."/>', description: 'Define a property/constant', attributes: 'name: string (required) - Property name\nvalue: string (required) - Property value (can be expression)' },
    { name: 'xacro:macro', snippet: 'xacro:macro name="$1" params="$2">\n\t$0\n</xacro:macro>', signature: '<xacro:macro name="..." params="param1 param2:=default *block">', description: 'Define a reusable macro', attributes: 'name: string (required) - Macro name\nparams: string (optional) - Space-separated parameters\n  - Simple: param\n  - Default value: param:=default\n  - Block: *block_name' },
    { name: 'xacro:insert_block', snippet: 'xacro:insert_block name="$1"/>', signature: '<xacro:insert_block name="..."/>', description: 'Insert a block parameter', attributes: 'name: string (required) - Name of block parameter to insert' },
    { name: 'xacro:include', snippet: 'xacro:include filename="$1"/>', signature: '<xacro:include filename="..."/>', description: 'Include another xacro file', attributes: 'filename: string (required) - Path to xacro file (supports package://)' },
    { name: 'xacro:if', snippet: 'xacro:if value="$1">\n\t$0\n</xacro:if>', signature: '<xacro:if value="${condition}">', description: 'Conditional inclusion', attributes: 'value: boolean expression (required) - Include content if true' },
    { name: 'xacro:unless', snippet: 'xacro:unless value="$1">\n\t$0\n</xacro:unless>', signature: '<xacro:unless value="${condition}">', description: 'Conditional exclusion', attributes: 'value: boolean expression (required) - Include content if false' },
    { name: 'xacro:arg', snippet: 'xacro:arg name="$1" default="$2"/>', signature: '<xacro:arg name="..." default="..."/>', description: 'Define command-line argument', attributes: 'name: string (required) - Argument name\ndefault: string (optional) - Default value if not provided' },
];

// Common attribute suggestions
const commonAttributes = {
    'link': ['name'],
    'joint': ['name', 'type'],
    'origin': ['xyz', 'rpy'],
    'geometry': [],
    'material': ['name'],
    'color': ['rgba'],
    'mesh': ['filename', 'scale'],
    'box': ['size'],
    'cylinder': ['radius', 'length'],
    'sphere': ['radius'],
    'limit': ['lower', 'upper', 'effort', 'velocity'],
    'axis': ['xyz'],
};

export class URDFXacroCompletionProvider implements vscode.CompletionItemProvider {
    provideCompletionItems(
        document: vscode.TextDocument,
        position: vscode.Position,
        token: vscode.CancellationToken,
        context: vscode.CompletionContext
    ): vscode.CompletionItem[] {
        const completions: vscode.CompletionItem[] = [];
        const line = document.lineAt(position.line);
        const lineText = line.text;
        const beforeCursor = lineText.substring(0, position.character);

        // Check if we're in an XML tag
        const inTag = beforeCursor.lastIndexOf('<') > beforeCursor.lastIndexOf('>');
        
        // Check if we're after a joint type attribute
        const inJointType = beforeCursor.includes('joint') && beforeCursor.includes('type=');

        if (inJointType && beforeCursor.endsWith('"')) {
            // Suggest joint types
            jointTypes.forEach(type => {
                const item = new vscode.CompletionItem(type, vscode.CompletionItemKind.EnumMember);
                item.documentation = new vscode.MarkdownString(`Joint type: ${type}`);
                completions.push(item);
            });
            return completions;
        }

        if (inTag || context.triggerCharacter === '<') {
            // Check if it's a Xacro file
            const isXacro = document.fileName.endsWith('.xacro');

            // Add URDF elements
            urdfElements.forEach(elem => {
                const item = new vscode.CompletionItem(elem.name, vscode.CompletionItemKind.Class);
                item.insertText = new vscode.SnippetString(elem.snippet);
                item.documentation = new vscode.MarkdownString(elem.description);
                item.sortText = `0_${elem.name}`;
                completions.push(item);
            });

            // Add geometry types
            geometryTypes.forEach(geom => {
                const item = new vscode.CompletionItem(geom.name, vscode.CompletionItemKind.Struct);
                item.insertText = new vscode.SnippetString(geom.snippet);
                item.documentation = new vscode.MarkdownString(geom.description);
                item.sortText = `1_${geom.name}`;
                completions.push(item);
            });

            // Add material elements
            materialElements.forEach(mat => {
                const item = new vscode.CompletionItem(mat.name, vscode.CompletionItemKind.Property);
                item.insertText = new vscode.SnippetString(mat.snippet);
                item.documentation = new vscode.MarkdownString(mat.description);
                item.sortText = `2_${mat.name}`;
                completions.push(item);
            });

            // Add Xacro elements if it's a Xacro file
            if (isXacro) {
                xacroElements.forEach(xacro => {
                    const item = new vscode.CompletionItem(xacro.name, vscode.CompletionItemKind.Snippet);
                    item.insertText = new vscode.SnippetString(xacro.snippet);
                    item.documentation = new vscode.MarkdownString(xacro.description);
                    item.sortText = `xacro_${xacro.name}`;
                    completions.push(item);
                });
            }
        }

        // Suggest Xacro property references when typing ${}
        if (beforeCursor.endsWith('${') || beforeCursor.includes('${') && !beforeCursor.endsWith('}')) {
            const properties = this.extractProperties(document);
            properties.forEach(prop => {
                const item = new vscode.CompletionItem(prop, vscode.CompletionItemKind.Variable);
                item.insertText = prop;
                item.documentation = new vscode.MarkdownString(`Xacro property: \${${prop}}`);
                completions.push(item);
            });
        }

        return completions;
    }

    private extractProperties(document: vscode.TextDocument): string[] {
        const properties: string[] = [];
        
        // Don't process overly large documents
        if (document.lineCount > 10000) {
            return properties;
        }
        
        const text = document.getText();
        const propertyRegex = /<xacro:property\s+name="([^"]+)"/g;
        let match;
        
        while ((match = propertyRegex.exec(text)) !== null) {
            properties.push(match[1]);
        }
        
        return properties;
    }
}

// Hover provider for URDF/Xacro
export class URDFXacroHoverProvider implements vscode.HoverProvider {
    async provideHover(
        document: vscode.TextDocument,
        position: vscode.Position,
        token: vscode.CancellationToken
    ): Promise<vscode.Hover | undefined> {
        // First check if we're hovering over a xacro property reference ${property_name}
        const line = document.lineAt(position.line).text;
        const propertyHover = await this.checkPropertyReference(document, position, line);
        if (propertyHover) {
            return propertyHover;
        }

        const range = document.getWordRangeAtPosition(position, /[\w:]+/);
        if (!range) {
            return undefined;
        }

        const word = document.getText(range);
        
        // Check for URDF elements
        const urdfElement = urdfElements.find(e => e.name === word);
        if (urdfElement) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(urdfElement.signature, 'xml');
            markdown.appendMarkdown('\n\n**' + urdfElement.description + '**\n\n');
            if (urdfElement.attributes) {
                markdown.appendMarkdown('### Attributes\n```\n' + urdfElement.attributes + '\n```');
            }
            return new vscode.Hover(markdown, range);
        }

        // Check for geometry types
        const geomElement = geometryTypes.find(g => g.name === word);
        if (geomElement) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(geomElement.signature, 'xml');
            markdown.appendMarkdown('\n\n**' + geomElement.description + '**\n\n');
            if (geomElement.attributes) {
                markdown.appendMarkdown('### Attributes\n```\n' + geomElement.attributes + '\n```');
            }
            return new vscode.Hover(markdown, range);
        }

        // Check for material elements
        const matElement = materialElements.find(m => m.name === word);
        if (matElement) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(matElement.signature, 'xml');
            markdown.appendMarkdown('\n\n**' + matElement.description + '**\n\n');
            if (matElement.attributes) {
                markdown.appendMarkdown('### Attributes\n```\n' + matElement.attributes + '\n```');
            }
            return new vscode.Hover(markdown, range);
        }

        // Check for Xacro elements
        const xacroElement = xacroElements.find(x => x.name === word);
        if (xacroElement) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(xacroElement.signature, 'xml');
            markdown.appendMarkdown('\n\n**' + xacroElement.description + '**\n\n');
            if (xacroElement.attributes) {
                markdown.appendMarkdown('### Attributes\n```\n' + xacroElement.attributes + '\n```');
            }
            return new vscode.Hover(markdown, range);
        }

        // Check for joint types
        if (jointTypes.includes(word)) {
            const descriptions: { [key: string]: string } = {
                'revolute': '**Revolute Joint**\n\nA hinge joint that rotates along an axis with limits.\n\n**Required:** `<limit>` element with lower, upper, effort, and velocity',
                'continuous': '**Continuous Joint**\n\nA continuous hinge joint that rotates around an axis without limits.\n\n**Note:** No `<limit>` element required',
                'prismatic': '**Prismatic Joint**\n\nA sliding joint that translates along an axis.\n\n**Required:** `<limit>` element with lower, upper, effort, and velocity',
                'fixed': '**Fixed Joint**\n\nA joint with no degrees of freedom. The child is rigidly attached to the parent.\n\n**Note:** No axis or limit elements needed',
                'floating': '**Floating Joint**\n\nA joint with 6 degrees of freedom (3 translation + 3 rotation).\n\n**Note:** Rarely used in practice',
                'planar': '**Planar Joint**\n\nA joint that allows motion in a plane perpendicular to the axis.\n\n**Note:** Has 2 translation and 1 rotation degrees of freedom'
            };
            const markdown = new vscode.MarkdownString(descriptions[word]);
            return new vscode.Hover(markdown, range);
        }

        // Check for common attributes
        const attributeDocs: { [key: string]: string } = {
            'xyz': '**xyz attribute**\n\nSpecifies x, y, z coordinates in meters.\n\nFormat: `xyz="x y z"` (space-separated values)',
            'rpy': '**rpy attribute**\n\nSpecifies roll, pitch, yaw rotations in radians.\n\nFormat: `rpy="roll pitch yaw"` (space-separated values)\n\n**Note:** Rotations applied in order: yaw (Z), pitch (Y), roll (X)',
            'rgba': '**rgba attribute**\n\nSpecifies red, green, blue, alpha color values.\n\nFormat: `rgba="r g b a"` (values 0.0 to 1.0)\n\nExample: `rgba="1.0 0.0 0.0 1.0"` for opaque red',
            'filename': '**filename attribute**\n\nPath to a file resource.\n\n**Supported formats:**\n- `package://package_name/path/to/file`\n- `file:///absolute/path/to/file`\n- Relative paths from URDF location\n\n**Mesh formats:** STL, DAE (COLLADA), OBJ, GLB, GLTF',
            'scale': '**scale attribute**\n\nScaling factors for mesh geometry.\n\nFormat: `scale="x y z"` (space-separated multipliers)\n\nExample: `scale="0.001 0.001 0.001"` to convert mm to meters',
            'effort': '**effort attribute**\n\nMaximum force (N) or torque (Nm) that can be applied by the joint.\n\nUsed by controllers and physics simulators.',
            'velocity': '**velocity attribute**\n\nMaximum velocity (m/s or rad/s) of the joint.\n\nUsed by controllers and motion planners.',
            'lower': '**lower attribute**\n\nLower limit of joint range.\n\n- **Revolute/continuous:** radians\n- **Prismatic:** meters',
            'upper': '**upper attribute**\n\nUpper limit of joint range.\n\n- **Revolute/continuous:** radians\n- **Prismatic:** meters'
        };

        if (attributeDocs[word]) {
            const markdown = new vscode.MarkdownString(attributeDocs[word]);
            return new vscode.Hover(markdown, range);
        }

        return undefined;
    }

    private async checkPropertyReference(
        document: vscode.TextDocument,
        position: vscode.Position,
        line: string
    ): Promise<vscode.Hover | undefined> {
        const charPos = position.character;
        
        // Search backward to find the start of ${
        let startPos = -1;
        for (let i = charPos; i >= 1; i--) {
            if (line[i] === '{' && line[i - 1] === '$') {
                startPos = i - 1;
                break;
            }
            // If we hit a closing brace before finding ${, we're not in a property reference
            if (line[i] === '}') {
                return undefined;
            }
        }
        
        // No ${ found before cursor
        if (startPos === -1) {
            return undefined;
        }
        
        // Search forward for the closing }
        let endPos = -1;
        for (let i = startPos + 2; i < line.length; i++) {
            if (line[i] === '}') {
                endPos = i;
                break;
            }
        }
        
        // No closing brace found or cursor is after the closing brace
        if (endPos === -1 || charPos > endPos) {
            return undefined;
        }
        
        // Extract property name (everything between ${ and })
        const propertyName = line.substring(startPos + 2, endPos).trim();
        
        // Handle property expressions like ${property_name} or ${property/2}
        // For now, just extract the base property name (before any operators)
        const basePropertyName = propertyName.split(/[\s\+\-\*\/\(\)]/)[0].trim();
        
        if (!basePropertyName) {
            return undefined;
        }
        
        // Find the property definition in the document and included files
        const propertyInfo = await this.findPropertyDefinition(document, basePropertyName);
        if (propertyInfo) {
            const markdown = new vscode.MarkdownString();
            markdown.appendMarkdown(`Value: \`${propertyInfo.value}\``);
            
            if (propertyName !== basePropertyName) {
                markdown.appendMarkdown(`\n\nUsed in expression: \`${propertyName}\``);
            }
            
            if (propertyInfo.sourceFile) {
                markdown.appendMarkdown(`\n\n*Defined in: ${propertyInfo.sourceFile}*`);
            }
            
            const hoverRange = new vscode.Range(
                position.line,
                startPos,
                position.line,
                endPos + 1
            );
            return new vscode.Hover(markdown, hoverRange);
        }
        
        return undefined;
    }

    private async findPropertyDefinition(
        document: vscode.TextDocument,
        propertyName: string
    ): Promise<{ value: string; sourceFile?: string } | undefined> {
        // Search for property definition in current file first
        const propertyInfo = this.searchPropertyInFile(document, propertyName);
        if (propertyInfo) {
            return propertyInfo;
        }
        
        // Search in included files
        const includedFiles = await this.findIncludedFiles(document);
        tracing.appendLine(`[findPropertyDefinition] Included files: ${JSON.stringify(includedFiles)}`);
        for (const filePath of includedFiles) {
            try {
                tracing.appendLine(`[findPropertyDefinition] Searching in: ${filePath}`);
                const includedDoc = await vscode.workspace.openTextDocument(filePath);
                const propertyInfo = this.searchPropertyInFile(includedDoc, propertyName);
                if (propertyInfo) {
                    tracing.appendLine(`[findPropertyDefinition] Found property in included file: ${filePath}`);
                    // Add the source file information
                    return {
                        value: propertyInfo.value,
                        sourceFile: path.basename(filePath)
                    };
                }
            } catch (error) {
                // Continue searching other files
            }
        }
        
        return undefined;
    }

    private searchPropertyInFile(
        document: vscode.TextDocument,
        propertyName: string
    ): { value: string; sourceFile?: string } | undefined {
        // Don't process overly large documents
        if (document.lineCount > 10000) {
            return undefined;
        }
        
        const text = document.getText();
        
        // Search for property definition: <xacro:property name="propertyName" value="..." />
        const propertyRegex = new RegExp(
            `<xacro:property\\s+name=["']${propertyName.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}["']\\s+value=["']([^"']+)["']`,
            'i'
        );
        
        const match = text.match(propertyRegex);
        if (match) {
            return {
                value: match[1],
                sourceFile: undefined
            };
        }
        
        return undefined;
    }

    private async findIncludedFiles(document: vscode.TextDocument): Promise<string[]> {
        const files: string[] = [];
        
        // Don't process overly large documents
        if (document.lineCount > 10000) {
            tracing.appendLine(`[findIncludedFiles] Skipping large document (${document.lineCount} lines): ${document.uri.fsPath}`);
            return files;
        }
        
        const text = document.getText();
        const documentDir = path.dirname(document.uri.fsPath);
        
        tracing.appendLine(`[findIncludedFiles] Searching for includes in: ${document.uri.fsPath}`);
        tracing.appendLine(`[findIncludedFiles] Document directory: ${documentDir}`);
        
        // Match <xacro:include filename="..." /> - includes $(find package) syntax and other patterns
        const includePattern = /<xacro:include\s+filename=["']([^"']+)["']/g;
        let match;
        
        while ((match = includePattern.exec(text)) !== null) {
            let includedFile = match[1];
            tracing.appendLine(`[findIncludedFiles] Found include: ${includedFile}`);
            
            // Handle $(find package_name)/path syntax
            const findMatch = includedFile.match(/\$\(find\s+([\w-]+)\)\/?(.*)/);
            if (findMatch) {
                const packageName = findMatch[1];
                const relativePath = findMatch[2] || '';
                tracing.appendLine(`[findIncludedFiles] Detected $(find ${packageName}) syntax, path: ${relativePath}`);
                const resolved = await this.resolvePackagePath(packageName, relativePath);
                if (resolved) {
                    files.push(resolved);
                    tracing.appendLine(`[findIncludedFiles] Added package file: ${resolved}`);
                }
                continue;
            }
            
            // Try relative to current file
            let fullPath = path.resolve(documentDir, includedFile);
            tracing.appendLine(`[findIncludedFiles] Trying path: ${fullPath}`);
            tracing.appendLine(`[findIncludedFiles] File exists: ${fs.existsSync(fullPath)}`);
            if (fs.existsSync(fullPath)) {
                files.push(fullPath);
                tracing.appendLine(`[findIncludedFiles] Added file: ${fullPath}`);
                continue;
            }
            
            // Try resolving as package path (package_name/path)
            const packageMatch = includedFile.match(/^([\w-]+)\/(.*)/);
            if (packageMatch) {
                const resolved = await this.resolvePackagePath(packageMatch[1], packageMatch[2]);
                if (resolved) {
                    files.push(resolved);
                    tracing.appendLine(`[findIncludedFiles] Added package file: ${resolved}`);
                }
            }
        }
        
        tracing.appendLine(`[findIncludedFiles] Total files found: ${files.length}`);
        return files;
    }

    private async resolvePackagePath(packageName: string, relativePath: string): Promise<string | undefined> {
        tracing.appendLine(`[resolvePackagePath] Looking for package: ${packageName}, path: ${relativePath}`);
        
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
                    
                    tracing.appendLine(`[resolvePackagePath] Found package at: ${packageDir}`);
                    tracing.appendLine(`[resolvePackagePath] Full path: ${fullPath}`);
                    
                    if (fs.existsSync(fullPath)) {
                        return fullPath;
                    }
                }
            } catch (error) {
                // Continue searching
            }
        }
        
        tracing.appendLine(`[resolvePackagePath] Package not found: ${packageName}`);
        return undefined;
    }
}
