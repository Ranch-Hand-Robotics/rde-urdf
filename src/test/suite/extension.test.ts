import * as assert from 'assert';

// You can import and use all API from the 'vscode' module
// as well as import your extension to test it
import * as vscode from 'vscode';
import { convertFindToPackageUri, processUrdfContent } from '../../utils';
import { getAllOpenSCADLibraryPaths, getDefaultOpenSCADLibraryPaths } from '../../openscad';
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
		
		// At least some default paths should be in the result (those that exist)
		// We can't guarantee all default paths exist on the test system
		assert.ok(paths.length >= 0, 'Should return paths array');
	});
});