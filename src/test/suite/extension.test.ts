import * as assert from 'assert';

// You can import and use all API from the 'vscode' module
// as well as import your extension to test it
import * as vscode from 'vscode';
import { convertFindToPackageUri, processUrdfContent } from '../../utils';

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