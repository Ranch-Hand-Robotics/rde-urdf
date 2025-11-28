// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// URDF and Xacro IntelliSense completion provider

import * as vscode from 'vscode';

// URDF elements
const urdfElements = [
    { name: 'robot', snippet: 'robot name="$1">\n\t$0\n</robot>', description: 'Root element of a URDF file' },
    { name: 'link', snippet: 'link name="$1">\n\t$0\n</link>', description: 'Defines a rigid body with inertia, visual, and collision properties' },
    { name: 'joint', snippet: 'joint name="$1" type="$2">\n\t<parent link="$3"/>\n\t<child link="$4"/>\n\t$0\n</joint>', description: 'Connects two links together' },
    { name: 'visual', snippet: 'visual>\n\t<geometry>\n\t\t$0\n\t</geometry>\n</visual>', description: 'Visual representation of a link' },
    { name: 'collision', snippet: 'collision>\n\t<geometry>\n\t\t$0\n\t</geometry>\n</collision>', description: 'Collision shape for physics simulation' },
    { name: 'inertial', snippet: 'inertial>\n\t<origin xyz="0 0 0" rpy="0 0 0"/>\n\t<mass value="$1"/>\n\t<inertia ixx="$2" ixy="0" ixz="0" iyy="$3" iyz="0" izz="$4"/>\n</inertial>', description: 'Inertial properties of a link' },
    { name: 'geometry', snippet: 'geometry>\n\t$0\n</geometry>', description: 'Geometric shape definition' },
    { name: 'origin', snippet: 'origin xyz="$1" rpy="$2"/>', description: 'Position and orientation (xyz in meters, rpy in radians)' },
    { name: 'parent', snippet: 'parent link="$1"/>', description: 'Parent link in a joint' },
    { name: 'child', snippet: 'child link="$1"/>', description: 'Child link in a joint' },
    { name: 'axis', snippet: 'axis xyz="$1"/>', description: 'Axis of rotation or translation for a joint' },
    { name: 'limit', snippet: 'limit lower="$1" upper="$2" effort="$3" velocity="$4"/>', description: 'Joint limits' },
    { name: 'dynamics', snippet: 'dynamics damping="$1" friction="$2"/>', description: 'Physical properties for joint motion' },
    { name: 'calibration', snippet: 'calibration rising="$1" falling="$2"/>', description: 'Joint calibration reference positions' },
    { name: 'safety_controller', snippet: 'safety_controller soft_lower_limit="$1" soft_upper_limit="$2" k_position="$3" k_velocity="$4"/>', description: 'Safety limits and gains' },
    { name: 'mimic', snippet: 'mimic joint="$1" multiplier="$2" offset="$3"/>', description: 'Makes this joint mimic another joint' },
];

// Geometry types
const geometryTypes = [
    { name: 'box', snippet: 'box size="$1"/>', description: 'Rectangular box (size="x y z" in meters)' },
    { name: 'cylinder', snippet: 'cylinder radius="$1" length="$2"/>', description: 'Cylinder shape' },
    { name: 'sphere', snippet: 'sphere radius="$1"/>', description: 'Sphere shape' },
    { name: 'mesh', snippet: 'mesh filename="$1" scale="1 1 1"/>', description: 'Load 3D mesh file (STL, DAE, OBJ, etc.)' },
];

// Material elements
const materialElements = [
    { name: 'material', snippet: 'material name="$1">\n\t<color rgba="$2"/>\n</material>', description: 'Material definition with color' },
    { name: 'color', snippet: 'color rgba="$1"/>', description: 'Color in RGBA format (values 0-1)' },
    { name: 'texture', snippet: 'texture filename="$1"/>', description: 'Texture image file' },
];

// Joint types
const jointTypes = [
    'revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar'
];

// Xacro-specific elements
const xacroElements = [
    { name: 'xacro:property', snippet: 'xacro:property name="$1" value="$2"/>', description: 'Define a property/constant' },
    { name: 'xacro:macro', snippet: 'xacro:macro name="$1" params="$2">\n\t$0\n</xacro:macro>', description: 'Define a reusable macro' },
    { name: 'xacro:insert_block', snippet: 'xacro:insert_block name="$1"/>', description: 'Insert a block parameter' },
    { name: 'xacro:include', snippet: 'xacro:include filename="$1"/>', description: 'Include another xacro file' },
    { name: 'xacro:if', snippet: 'xacro:if value="$1">\n\t$0\n</xacro:if>', description: 'Conditional inclusion' },
    { name: 'xacro:unless', snippet: 'xacro:unless value="$1">\n\t$0\n</xacro:unless>', description: 'Conditional exclusion' },
    { name: 'xacro:arg', snippet: 'xacro:arg name="$1" default="$2"/>', description: 'Define command-line argument' },
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
    provideHover(
        document: vscode.TextDocument,
        position: vscode.Position,
        token: vscode.CancellationToken
    ): vscode.Hover | undefined {
        const range = document.getWordRangeAtPosition(position, /[\w:]+/);
        if (!range) {
            return undefined;
        }

        const word = document.getText(range);
        
        // Check for URDF elements
        const urdfElement = urdfElements.find(e => e.name === word);
        if (urdfElement) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`<${urdfElement.name}>`, 'xml');
            markdown.appendMarkdown('\n\n' + urdfElement.description);
            return new vscode.Hover(markdown, range);
        }

        // Check for geometry types
        const geomElement = geometryTypes.find(g => g.name === word);
        if (geomElement) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`<${geomElement.name}>`, 'xml');
            markdown.appendMarkdown('\n\n' + geomElement.description);
            return new vscode.Hover(markdown, range);
        }

        // Check for material elements
        const matElement = materialElements.find(m => m.name === word);
        if (matElement) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`<${matElement.name}>`, 'xml');
            markdown.appendMarkdown('\n\n' + matElement.description);
            return new vscode.Hover(markdown, range);
        }

        // Check for Xacro elements
        const xacroElement = xacroElements.find(x => x.name === word);
        if (xacroElement) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`<${xacroElement.name}>`, 'xml');
            markdown.appendMarkdown('\n\n' + xacroElement.description);
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
}
