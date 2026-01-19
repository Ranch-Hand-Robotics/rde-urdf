import * as assert from 'assert';

// You can import and use all API from the 'vscode' module
// as well as import your extension to test it
import * as vscode from 'vscode';
import { convertFindToPackageUri, processUrdfContent } from '../../utils';
import { validateOpenSCAD } from '../../openscad';
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