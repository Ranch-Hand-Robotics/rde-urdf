import * as assert from 'assert';
import * as path from 'path';
import * as fs from 'fs';
import * as xacroConfig from '../xacroConfig';

suite('Xacro Configuration Test Suite', () => {
    const testDataPath = path.join(__dirname, '../../src/test/testdata');

    test('Detect arguments in xacro file', () => {
        const filePath = path.join(testDataPath, 'robot_with_args.xacro');
        const content = fs.readFileSync(filePath, 'utf8');
        
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.strictEqual(detected.hasArgs, true, 'Should detect arguments');
        assert.ok(detected.argNames.includes('robot_name'), 'Should detect robot_name argument');
        assert.ok(detected.argNames.includes('base_size'), 'Should detect base_size argument');
        assert.ok(detected.argNames.includes('wheel_count'), 'Should detect wheel_count argument');
    });

    test('Detect environment variables in xacro file', () => {
        const filePath = path.join(testDataPath, 'robot_with_env.xacro');
        const content = fs.readFileSync(filePath, 'utf8');
        
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.strictEqual(detected.hasEnv, true, 'Should detect environment variables');
        assert.ok(detected.envNames.includes('ROBOT_COLOR'), 'Should detect ROBOT_COLOR env var');
        assert.ok(detected.envNames.includes('ROBOT_PREFIX'), 'Should detect ROBOT_PREFIX env var');
    });

    test('Detect both arguments and environment variables', () => {
        const filePath = path.join(testDataPath, 'robot_with_args_and_env.xacro');
        const content = fs.readFileSync(filePath, 'utf8');
        
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.strictEqual(detected.hasArgs, true, 'Should detect arguments');
        assert.strictEqual(detected.hasEnv, true, 'Should detect environment variables');
        assert.ok(detected.argNames.includes('robot_name'), 'Should detect robot_name argument');
        assert.ok(detected.envNames.includes('ROBOT_PREFIX'), 'Should detect ROBOT_PREFIX env var');
    });

    test('Match file with exact path', () => {
        const config: xacroConfig.XacroConfig = {
            version: '1.0.0',
            '/path/to/robot.xacro': {
                args: { robot_name: 'test_robot' }
            }
        };

        const fileConfig = xacroConfig.findConfigForFile('/path/to/robot.xacro', config);
        
        assert.ok(fileConfig, 'Should find config for exact path');
        assert.strictEqual(fileConfig?.args?.robot_name, 'test_robot');
    });

    test('Match file with wildcard pattern', () => {
        const config: xacroConfig.XacroConfig = {
            version: '1.0.0',
            '**/*.xacro': {
                args: { default_arg: 'value' }
            }
        };

        const fileConfig = xacroConfig.findConfigForFile('/any/path/to/robot.xacro', config);
        
        assert.ok(fileConfig, 'Should find config for wildcard pattern');
        assert.strictEqual(fileConfig?.args?.default_arg, 'value');
    });

    test('No detection on file without arguments or env vars', () => {
        const filePath = path.join(testDataPath, 'simple_box.urdf');
        const content = fs.readFileSync(filePath, 'utf8');
        
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.strictEqual(detected.hasArgs, false, 'Should not detect arguments');
        assert.strictEqual(detected.hasEnv, false, 'Should not detect environment variables');
    });
});
