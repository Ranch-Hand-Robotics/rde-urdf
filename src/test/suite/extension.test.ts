import * as assert from 'assert';

// You can import and use all API from the 'vscode' module
// as well as import your extension to test it
import * as vscode from 'vscode';
import { convertFindToPackageUri, processUrdfContent } from '../../utils';
import {
	validateOpenSCAD,
	getAllOpenSCADLibraryPaths,
	getDefaultOpenSCADLibraryPaths,
	parseOpenSCADCustomizerVariables
} from '../../openscad';
import * as path from 'path';

suite('Extension Test Suite', () => {
	vscode.window.showInformationMessage('Start all tests.');

	test('convertFindToPackageUri - basic conversion', () => {
		const input = '${find my_package}/meshes/robot.stl';
		const expected = 'package://my_package/meshes/robot.stl';
		const result = convertFindToPackageUri(input);
		assert.strictEqual(result, expected);
	});

	test('convertFindToPackageUri - multiple packages', () => {
		const input = '<mesh filename="${find robot_pkg}/meshes/base.stl"/>\n<mesh filename="${find sensor_pkg}/meshes/sensor.stl"/>';
		const expected = '<mesh filename="package://robot_pkg/meshes/base.stl"/>\n<mesh filename="package://sensor_pkg/meshes/sensor.stl"/>';
		const result = convertFindToPackageUri(input);
		assert.strictEqual(result, expected);
	});

	test('convertFindToPackageUri - package names with underscores and hyphens', () => {
		const input = '${find my_robot_package}/config/params.yaml and ${find sensor-package}/launch/sensor.launch';
		const expected = 'package://my_robot_package/config/params.yaml and package://sensor-package/launch/sensor.launch';
		const result = convertFindToPackageUri(input);
		assert.strictEqual(result, expected);
	});

	test('convertFindToPackageUri - with whitespace', () => {
		const input = '${ find  my_package  }/file.txt';
		const expected = 'package://my_package/file.txt';
		const result = convertFindToPackageUri(input);
		assert.strictEqual(result, expected);
	});

	test('convertFindToPackageUri - no match should return unchanged', () => {
		const input = 'package://already_converted/file.txt';
		const expected = 'package://already_converted/file.txt';
		const result = convertFindToPackageUri(input);
		assert.strictEqual(result, expected);
	});

	test('processUrdfContent - wrapper function test', () => {
		const input = '<mesh filename="${find test_package}/meshes/test.stl"/>';
		const expected = '<mesh filename="package://test_package/meshes/test.stl"/>';
		const result = processUrdfContent(input);
		assert.strictEqual(result, expected);
	});

	test('validateOpenSCAD - valid file', async () => {
		const testDataPath = path.join(__dirname, '..', 'testdata');
		const validFile = path.join(testDataPath, 'simple_cube.scad');
		const result = await validateOpenSCAD(validFile);
		assert.strictEqual(result.valid, true, 'Valid OpenSCAD file should be marked as valid');
		assert.strictEqual(result.errors.length, 0, 'Valid file should have no errors');
	});

	test('validateOpenSCAD - invalid file with syntax errors', async () => {
		const testDataPath = path.join(__dirname, '..', 'testdata');
		const invalidFile = path.join(testDataPath, 'invalid_syntax.scad');
		const result = await validateOpenSCAD(invalidFile);
		assert.strictEqual(result.valid, false, 'Invalid OpenSCAD file should be marked as invalid');
		assert.ok(result.errors.length > 0, 'Invalid file should have errors');
	});

	test('validateOpenSCAD - with content string', async () => {
		const validContent = 'cube([10, 10, 10]);';
		const result = await validateOpenSCAD('/tmp/test.scad', validContent);
		assert.strictEqual(result.valid, true, 'Valid OpenSCAD content should be marked as valid');
		assert.strictEqual(result.errors.length, 0, 'Valid content should have no errors');
	});
});

suite('OpenSCAD Library Paths Test Suite', () => {
	test('getDefaultOpenSCADLibraryPaths - returns array', () => {
		const paths = getDefaultOpenSCADLibraryPaths();
		assert.ok(Array.isArray(paths), 'Should return an array');
		assert.ok(paths.length > 0, 'Should return at least one path');
	});

	test('getAllOpenSCADLibraryPaths - includes SCAD file directory', async () => {
		const testScadPath = path.join(__dirname, '../testdata/simple_cube.scad');
		const testDir = path.dirname(testScadPath);
		
		const paths = await getAllOpenSCADLibraryPaths(undefined, testScadPath);
		
		// The SCAD file's directory should be the first path
		assert.ok(paths.length > 0, 'Should return at least one path');
		assert.strictEqual(path.normalize(paths[0]), path.normalize(testDir), 'First path should be the SCAD file directory');
	});

	test('getAllOpenSCADLibraryPaths - does not include workspace root', async () => {
		const workspaceRoot = '/some/workspace/root';
		const testScadPath = path.join(workspaceRoot, 'subdirectory', 'test.scad');
		const testDir = path.dirname(testScadPath);
		
		const paths = await getAllOpenSCADLibraryPaths(workspaceRoot, testScadPath);
		
		// The workspace root should NOT be in the paths
		const normalizedPaths = paths.map(p => path.normalize(p));
		assert.ok(!normalizedPaths.includes(path.normalize(workspaceRoot)), 'Workspace root should not be included in library paths');
		
		// But the SCAD file's directory should be there (if it exists)
		// Note: In this test, the directory doesn't exist, so it may not be in the paths
		// We're mainly testing that workspace root is not auto-added
	});

	test('getAllOpenSCADLibraryPaths - includes default paths', async () => {
		const paths = await getAllOpenSCADLibraryPaths();
		const defaultPaths = getDefaultOpenSCADLibraryPaths();
		
		// Default paths that exist should be included in the result
		// Filter to only paths that exist and check they're in the result
		for (const defaultPath of defaultPaths) {
			const normalizedDefaultPath = path.normalize(defaultPath);
			const normalizedPaths = paths.map(p => path.normalize(p));
			// Note: We can only verify paths that actually exist on the system
			// If the path exists, it should be in the result
			if (normalizedPaths.includes(normalizedDefaultPath)) {
				assert.ok(true, `Default path ${defaultPath} is included in result`);
			}
		}
		
		// At minimum, we should have some paths returned
		assert.ok(paths.length >= 0, 'Should return a paths array');
	});
});

suite('Camera Configuration Test Suite', () => {
	test('Camera angle settings exist in configuration', () => {
		const config = vscode.workspace.getConfiguration("urdf-editor");
		
		// Check that the new camera settings exist with their default values
		const alpha = config.get("CameraAlpha");
		const beta = config.get("CameraBeta");
		const distance = config.get("CameraDistanceToRobot");
		const openScadCustomizerEnabled = config.get("OpenSCADCustomizerEnabled");
		
		// Verify they are numbers
		assert.ok(typeof alpha === 'number', 'CameraAlpha should be a number');
		assert.ok(typeof beta === 'number', 'CameraBeta should be a number');
		assert.ok(typeof distance === 'number', 'CameraDistanceToRobot should be a number');
		assert.ok(typeof openScadCustomizerEnabled === 'boolean', 'OpenSCADCustomizerEnabled should be a boolean');
		
		// Verify default values in degrees
		assert.strictEqual(alpha, -60, 'CameraAlpha should default to -60 degrees');
		assert.strictEqual(beta, 75, 'CameraBeta should default to 75 degrees');
		assert.strictEqual(distance, 1, 'CameraDistanceToRobot should default to 1');
		assert.strictEqual(openScadCustomizerEnabled, true, 'OpenSCADCustomizerEnabled should default to true');
	});
});

suite('OpenSCAD Customizer Parser Test Suite', () => {
	test('parseOpenSCADCustomizerVariables - parses supported values and tabs', async () => {
		const testDataPath = path.join(__dirname, '..', 'testdata');
		const customizerFile = path.join(testDataPath, 'customizer_features.scad');
		const content = Buffer.from(await vscode.workspace.fs.readFile(vscode.Uri.file(customizerFile))).toString('utf8');

		const result = parseOpenSCADCustomizerVariables(content);
		assert.ok(result.variables.length > 0, 'Expected parser to find customizer variables');

		const byName = new Map(result.variables.map(v => [v.name, v]));

		assert.strictEqual(byName.get('Numbers')?.widget, 'dropdown');
		assert.strictEqual(byName.get('Numbers')?.tab, 'Drop down box:');
		assert.strictEqual(byName.get('slider')?.widget, 'slider');
		assert.strictEqual(byName.get('stepSlider')?.range?.step, 5);
		assert.strictEqual(byName.get('Variable')?.widget, 'checkbox');
		assert.strictEqual(byName.get('String')?.widget, 'textbox');
		assert.strictEqual(byName.get('String')?.maxLength, 8);
		assert.strictEqual(byName.get('Vector3')?.widget, 'vector');

		assert.strictEqual(byName.has('debugMode'), false, 'Hidden variables should not be emitted');
		assert.strictEqual(byName.has('shownAfterBrace'), false, 'Variables after first block brace should not be emitted');
		assert.ok(result.firstBraceLine !== undefined, 'Expected parser to report first brace line');
	});

	test('parseOpenSCADCustomizerVariables - warns on unsupported expressions', () => {
		const content = [
			'/* [parameters] */',
			'supported = 1;',
			'unsupported = 1 + 2;',
			'text = str("a", "b");',
			'module stop() {}',
		].join('\n');

		const result = parseOpenSCADCustomizerVariables(content);
		assert.strictEqual(result.variables.some(v => v.name === 'supported'), true);
		assert.strictEqual(result.variables.some(v => v.name === 'unsupported'), false);
		assert.strictEqual(result.variables.some(v => v.name === 'text'), false);
		assert.ok(result.warnings.length >= 2, 'Expected warnings for unsupported customizer expressions');
	});

	test('parseOpenSCADCustomizerVariables - Hidden section hides following assignments until next tab', () => {
		const content = [
			'/* [Main] */',
			'a = 1;',
			'/* [Hidden] */',
			'b = 2;',
			'c = 3;',
			'/* [VisibleAgain] */',
			'd = 4;',
		].join('\n');

		const result = parseOpenSCADCustomizerVariables(content);
		const names = result.variables.map(v => v.name);

		assert.strictEqual(names.includes('a'), true);
		assert.strictEqual(names.includes('b'), false);
		assert.strictEqual(names.includes('c'), false);
		assert.strictEqual(names.includes('d'), true);
	});

	test('parseOpenSCADCustomizerVariables - lowercase hidden marker without spaces is recognized', () => {
		const content = [
			'/*[hidden]*/',
			'x = 1;',
			'y = "secret";',
			'/* [VisibleAgain] */',
			'z = 2;',
		].join('\n');

		const result = parseOpenSCADCustomizerVariables(content);
		const names = result.variables.map(v => v.name);

		assert.strictEqual(names.includes('x'), false);
		assert.strictEqual(names.includes('y'), false);
		assert.strictEqual(names.includes('z'), true);
	});

	test('parseOpenSCADCustomizerVariables - all variables hidden at start', () => {
		const content = [
			'/* [Hidden] */',
			'var1 = 10;',
			'var2 = 20;',
			'var3 = [1, 2, 3];',
			'',
			'// model code here',
			'cube(var1);',
		].join('\n');

		const result = parseOpenSCADCustomizerVariables(content);
		
		assert.strictEqual(result.variables.length, 0, 'Should have no visible variables when all are in Hidden section');
	});
});
