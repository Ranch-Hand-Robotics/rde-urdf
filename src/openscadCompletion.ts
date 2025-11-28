// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// OpenSCAD IntelliSense completion provider

import * as vscode from 'vscode';

// OpenSCAD built-in modules
const openscadModules = [
    { name: 'cube', params: 'size=[x, y, z], center=false', description: 'Creates a cube with specified dimensions' },
    { name: 'sphere', params: 'r=radius, $fn=fragments', description: 'Creates a sphere with specified radius' },
    { name: 'cylinder', params: 'h=height, r=radius, r1=bottom_radius, r2=top_radius, center=false, $fn=fragments', description: 'Creates a cylinder or cone' },
    { name: 'circle', params: 'r=radius, $fn=fragments', description: 'Creates a 2D circle' },
    { name: 'square', params: 'size=[x, y], center=false', description: 'Creates a 2D square or rectangle' },
    { name: 'polygon', params: 'points=[[x1,y1], [x2,y2], ...], paths=[[p1,p2,...]]', description: 'Creates a 2D polygon from points' },
    { name: 'polyhedron', params: 'points=[[x,y,z],...], faces=[[p1,p2,p3],...]', description: 'Creates a 3D polyhedron from points and faces' },
    { name: 'text', params: 'text="string", size=10, font="Liberation Sans", halign="left", valign="baseline"', description: 'Creates 2D or 3D text' },
];

// OpenSCAD transformations
const openscadTransforms = [
    { name: 'translate', params: 'v=[x, y, z]', description: 'Moves children by specified vector' },
    { name: 'rotate', params: 'a=angle or a=[x_angle, y_angle, z_angle], v=[x, y, z]', description: 'Rotates children around specified axis' },
    { name: 'scale', params: 'v=[x, y, z]', description: 'Scales children by specified factors' },
    { name: 'mirror', params: 'v=[x, y, z]', description: 'Mirrors children across specified plane' },
    { name: 'resize', params: 'newsize=[x, y, z], auto=false', description: 'Resizes children to new size' },
    { name: 'multmatrix', params: 'm=[[...],[...],[...],[...]]', description: 'Applies transformation matrix to children' },
    { name: 'color', params: 'c=[r, g, b, a] or c="colorname" or c="#RRGGBB"', description: 'Sets color of children for preview and export' },
    { name: 'offset', params: 'r=radius, delta=offset', description: 'Offsets 2D shapes inward or outward' },
    { name: 'hull', params: '', description: 'Creates convex hull of children' },
    { name: 'minkowski', params: '', description: 'Performs Minkowski sum of children' },
];

// OpenSCAD boolean operations
const openscadBooleans = [
    { name: 'union', params: '', description: 'Combines all children into single object' },
    { name: 'difference', params: '', description: 'Subtracts children from first child' },
    { name: 'intersection', params: '', description: 'Creates intersection of all children' },
];

// OpenSCAD 2D to 3D
const openscad2Dto3D = [
    { name: 'linear_extrude', params: 'height=h, center=false, convexity=10, twist=degrees, slices=number, scale=factor', description: 'Extrudes 2D shape linearly into 3D' },
    { name: 'rotate_extrude', params: 'angle=360, convexity=10, $fn=fragments', description: 'Rotates 2D shape around Z-axis to create 3D' },
];

// OpenSCAD special variables
const openscadVariables = [
    { name: '$fn', description: 'Number of fragments for circles/spheres (minimum 3)' },
    { name: '$fa', description: 'Minimum angle for fragment (degrees)' },
    { name: '$fs', description: 'Minimum size of fragment (mm)' },
    { name: '$t', description: 'Animation parameter (0 to 1)' },
    { name: '$vpr', description: 'Viewport rotation angles [x, y, z]' },
    { name: '$vpt', description: 'Viewport translation [x, y, z]' },
    { name: '$vpd', description: 'Viewport camera distance' },
    { name: '$children', description: 'Number of module children' },
];

// OpenSCAD keywords
const openscadKeywords = [
    { name: 'module', description: 'Defines a reusable module' },
    { name: 'function', description: 'Defines a function that returns a value' },
    { name: 'if', description: 'Conditional statement' },
    { name: 'else', description: 'Else clause for if statement' },
    { name: 'for', description: 'Loop construct' },
    { name: 'let', description: 'Local variable assignment' },
    { name: 'each', description: 'Iterate over list elements' },
    { name: 'true', description: 'Boolean true value' },
    { name: 'false', description: 'Boolean false value' },
    { name: 'undef', description: 'Undefined value' },
    { name: 'include', description: 'Include another OpenSCAD file' },
    { name: 'use', description: 'Use modules from another file' },
];

// Common color names
const colorNames = [
    'red', 'green', 'blue', 'yellow', 'cyan', 'magenta', 'white', 'black',
    'orange', 'purple', 'pink', 'brown', 'gray', 'lime', 'navy', 'teal',
];

export class OpenSCADCompletionProvider implements vscode.CompletionItemProvider {
    provideCompletionItems(
        document: vscode.TextDocument,
        position: vscode.Position,
        token: vscode.CancellationToken,
        context: vscode.CompletionContext
    ): vscode.CompletionItem[] {
        const completions: vscode.CompletionItem[] = [];

        // Add modules
        openscadModules.forEach(mod => {
            const item = new vscode.CompletionItem(mod.name, vscode.CompletionItemKind.Function);
            item.detail = mod.params;
            item.documentation = new vscode.MarkdownString(mod.description);
            item.insertText = new vscode.SnippetString(`${mod.name}($1)$0`);
            completions.push(item);
        });

        // Add transformations
        openscadTransforms.forEach(trans => {
            const item = new vscode.CompletionItem(trans.name, vscode.CompletionItemKind.Function);
            item.detail = trans.params;
            item.documentation = new vscode.MarkdownString(trans.description);
            if (trans.name === 'color') {
                item.insertText = new vscode.SnippetString(`${trans.name}("$1")$0`);
            } else {
                item.insertText = new vscode.SnippetString(`${trans.name}($1)$0`);
            }
            completions.push(item);
        });

        // Add boolean operations
        openscadBooleans.forEach(bool => {
            const item = new vscode.CompletionItem(bool.name, vscode.CompletionItemKind.Operator);
            item.detail = bool.params || 'Boolean operation';
            item.documentation = new vscode.MarkdownString(bool.description);
            item.insertText = new vscode.SnippetString(`${bool.name}() {\n\t$0\n}`);
            completions.push(item);
        });

        // Add 2D to 3D operations
        openscad2Dto3D.forEach(op => {
            const item = new vscode.CompletionItem(op.name, vscode.CompletionItemKind.Function);
            item.detail = op.params;
            item.documentation = new vscode.MarkdownString(op.description);
            item.insertText = new vscode.SnippetString(`${op.name}($1) {\n\t$0\n}`);
            completions.push(item);
        });

        // Add special variables
        openscadVariables.forEach(v => {
            const item = new vscode.CompletionItem(v.name, vscode.CompletionItemKind.Variable);
            item.documentation = new vscode.MarkdownString(v.description);
            completions.push(item);
        });

        // Add keywords
        openscadKeywords.forEach(kw => {
            const item = new vscode.CompletionItem(kw.name, vscode.CompletionItemKind.Keyword);
            item.documentation = new vscode.MarkdownString(kw.description);
            completions.push(item);
        });

        // Check if we're inside a color() call to suggest color names
        const lineText = document.lineAt(position.line).text;
        const beforeCursor = lineText.substring(0, position.character);
        if (beforeCursor.includes('color(') && !beforeCursor.includes(')')) {
            colorNames.forEach(color => {
                const item = new vscode.CompletionItem(color, vscode.CompletionItemKind.Color);
                item.insertText = `"${color}"`;
                item.documentation = new vscode.MarkdownString(`Color: ${color}`);
                completions.push(item);
            });
        }

        return completions;
    }
}

export class OpenSCADHoverProvider implements vscode.HoverProvider {
    provideHover(
        document: vscode.TextDocument,
        position: vscode.Position,
        token: vscode.CancellationToken
    ): vscode.Hover | undefined {
        const range = document.getWordRangeAtPosition(position);
        if (!range) {
            return undefined;
        }

        const word = document.getText(range);

        // Check modules
        const module = openscadModules.find(m => m.name === word);
        if (module) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`${module.name}(${module.params})`, 'openscad');
            markdown.appendMarkdown('\n\n' + module.description);
            return new vscode.Hover(markdown, range);
        }

        // Check transformations
        const transform = openscadTransforms.find(t => t.name === word);
        if (transform) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`${transform.name}(${transform.params})`, 'openscad');
            markdown.appendMarkdown('\n\n' + transform.description);
            return new vscode.Hover(markdown, range);
        }

        // Check boolean operations
        const boolean = openscadBooleans.find(b => b.name === word);
        if (boolean) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`${boolean.name}()`, 'openscad');
            markdown.appendMarkdown('\n\n' + boolean.description);
            return new vscode.Hover(markdown, range);
        }

        // Check 2D to 3D operations
        const op2d3d = openscad2Dto3D.find(o => o.name === word);
        if (op2d3d) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(`${op2d3d.name}(${op2d3d.params})`, 'openscad');
            markdown.appendMarkdown('\n\n' + op2d3d.description);
            return new vscode.Hover(markdown, range);
        }

        // Check special variables
        const variable = openscadVariables.find(v => v.name === word);
        if (variable) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(variable.name, 'openscad');
            markdown.appendMarkdown('\n\n' + variable.description);
            return new vscode.Hover(markdown, range);
        }

        // Check keywords
        const keyword = openscadKeywords.find(k => k.name === word);
        if (keyword) {
            const markdown = new vscode.MarkdownString();
            markdown.appendCodeblock(keyword.name, 'openscad');
            markdown.appendMarkdown('\n\n' + keyword.description);
            return new vscode.Hover(markdown, range);
        }

        return undefined;
    }
}
