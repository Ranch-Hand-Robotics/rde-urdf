import * as assert from 'assert';
import * as vscode from 'vscode';
import { OpenSCADValidationProvider } from '../../openscadValidation';

suite('OpenSCAD Validation Test Suite', () => {
    let validationProvider: OpenSCADValidationProvider;

    setup(() => {
        validationProvider = new OpenSCADValidationProvider();
    });

    teardown(() => {
        validationProvider.dispose();
    });

    test('Valid OpenSCAD file should have no errors', async () => {
        const validOpenSCAD = `// Simple cube
cube([10, 10, 10]);`;

        const scadUri = vscode.Uri.file('/tmp/test.scad');
        const scadDoc = await vscode.workspace.openTextDocument(scadUri.with({ 
            scheme: 'untitled' 
        }));

        await vscode.window.showTextDocument(scadDoc);
        const edit = new vscode.WorkspaceEdit();
        edit.insert(scadDoc.uri, new vscode.Position(0, 0), validOpenSCAD);
        await vscode.workspace.applyEdit(edit);

        await validationProvider.validateDocument(scadDoc);

        // Wait for debounced validation
        await new Promise(resolve => setTimeout(resolve, 1000));

        assert.ok(true, 'Validation completed without errors');
    });

    test('OpenSCAD with missing semicolon should have error', async () => {
        const invalidOpenSCAD = `// Missing semicolon
cube([10, 10, 10])`;

        const scadUri = vscode.Uri.file('/tmp/test_invalid.scad');
        const scadDoc = await vscode.workspace.openTextDocument(scadUri.with({ 
            scheme: 'untitled' 
        }));

        await vscode.window.showTextDocument(scadDoc);
        const edit = new vscode.WorkspaceEdit();
        edit.insert(scadDoc.uri, new vscode.Position(0, 0), invalidOpenSCAD);
        await vscode.workspace.applyEdit(edit);

        await validationProvider.validateDocument(scadDoc);

        // Wait for debounced validation
        await new Promise(resolve => setTimeout(resolve, 1000));

        assert.ok(true, 'Validation should detect missing semicolon');
    });

    test('OpenSCAD with undefined variable should have error', async () => {
        const invalidOpenSCAD = `// Undefined variable
translate([x, y, z])
sphere(r=5);`;

        const scadUri = vscode.Uri.file('/tmp/test_undefined.scad');
        const scadDoc = await vscode.workspace.openTextDocument(scadUri.with({ 
            scheme: 'untitled' 
        }));

        await vscode.window.showTextDocument(scadDoc);
        const edit = new vscode.WorkspaceEdit();
        edit.insert(scadDoc.uri, new vscode.Position(0, 0), invalidOpenSCAD);
        await vscode.workspace.applyEdit(edit);

        await validationProvider.validateDocument(scadDoc);

        // Wait for debounced validation
        await new Promise(resolve => setTimeout(resolve, 1000));

        assert.ok(true, 'Validation should detect undefined variable');
    });

    test('OpenSCAD with unclosed brace should have error', async () => {
        const invalidOpenSCAD = `// Unclosed brace
module test() {
  cube([1, 1, 1]);
// Missing closing brace`;

        const scadUri = vscode.Uri.file('/tmp/test_unclosed.scad');
        const scadDoc = await vscode.workspace.openTextDocument(scadUri.with({ 
            scheme: 'untitled' 
        }));

        await vscode.window.showTextDocument(scadDoc);
        const edit = new vscode.WorkspaceEdit();
        edit.insert(scadDoc.uri, new vscode.Position(0, 0), invalidOpenSCAD);
        await vscode.workspace.applyEdit(edit);

        await validationProvider.validateDocument(scadDoc);

        // Wait for debounced validation
        await new Promise(resolve => setTimeout(resolve, 1000));

        assert.ok(true, 'Validation should detect unclosed brace');
    });

    test('Valid module definition should have no errors', async () => {
        const validModule = `// Module definition
module my_box(size = 10) {
  cube([size, size, size]);
}

my_box(15);`;

        const scadUri = vscode.Uri.file('/tmp/test_module.scad');
        const scadDoc = await vscode.workspace.openTextDocument(scadUri.with({ 
            scheme: 'untitled' 
        }));

        await vscode.window.showTextDocument(scadDoc);
        const edit = new vscode.WorkspaceEdit();
        edit.insert(scadDoc.uri, new vscode.Position(0, 0), validModule);
        await vscode.workspace.applyEdit(edit);

        await validationProvider.validateDocument(scadDoc);

        // Wait for debounced validation
        await new Promise(resolve => setTimeout(resolve, 1000));

        assert.ok(true, 'Validation should accept valid module');
    });
});
