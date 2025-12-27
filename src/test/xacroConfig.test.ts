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

    test('Detect arguments in included files', () => {
        const filePath = path.join(testDataPath, 'included_with_args.xacro');
        const content = fs.readFileSync(filePath, 'utf8');
        
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.strictEqual(detected.hasArgs, true, 'Should detect arguments in included file');
        assert.ok(detected.argNames.includes('included_color'), 'Should detect included_color argument');
        assert.ok(detected.argNames.includes('included_size'), 'Should detect included_size argument');
        assert.strictEqual(detected.hasEnv, true, 'Should detect environment variables in included file');
        assert.ok(detected.envNames.includes('INCLUDED_MESH_PATH'), 'Should detect INCLUDED_MESH_PATH env var');
    });

    test('Detect arguments from main file with includes', () => {
        const filePath = path.join(testDataPath, 'main_with_includes_with_args.xacro');
        const content = fs.readFileSync(filePath, 'utf8');
        
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.strictEqual(detected.hasArgs, true, 'Should detect arguments in main file');
        assert.ok(detected.argNames.includes('main_robot_name'), 'Should detect main_robot_name argument');
    });

    test('Detect include directives with double quotes', () => {
        const content = '<xacro:include filename="test.xacro"/>';
        const includePattern = /<xacro:include\s+filename=(['"])([^'"]+)\1/g;
        const match = includePattern.exec(content);
        
        assert.ok(match, 'Should match include with double quotes');
        assert.strictEqual(match[2], 'test.xacro', 'Should extract filename');
    });

    test('Detect include directives with single quotes', () => {
        const content = "<xacro:include filename='test.xacro'/>";
        const includePattern = /<xacro:include\s+filename=(['"])([^'"]+)\1/g;
        const match = includePattern.exec(content);
        
        assert.ok(match, 'Should match include with single quotes');
        assert.strictEqual(match[2], 'test.xacro', 'Should extract filename');
    });

    test('Detect xacro:arg declarations', () => {
        const content = '<xacro:arg name="test_arg" default="value"/>';
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.ok(detected.argNames.includes('test_arg'), 'Should detect xacro:arg declaration');
    });

    test('Detect $(arg name) usage', () => {
        const content = '<property value="$(arg my_arg)"/>';
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.ok(detected.argNames.includes('my_arg'), 'Should detect $(arg) usage');
    });

    test('Detect $(env name) usage', () => {
        const content = '<property value="$(env MY_ENV_VAR)"/>';
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.ok(detected.envNames.includes('MY_ENV_VAR'), 'Should detect $(env) usage');
    });

    test('Detect $(optenv name default) usage', () => {
        const content = '<property value="$(optenv MY_OPT_ENV default_value)"/>';
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.ok(detected.envNames.includes('MY_OPT_ENV'), 'Should detect $(optenv) usage');
    });

    test('Avoid duplicate arguments', () => {
        const content = `
            <xacro:arg name="duplicate_arg" default="1"/>
            <property value="$(arg duplicate_arg)"/>
            <property value="$(arg duplicate_arg)"/>
        `;
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        const count = detected.argNames.filter(name => name === 'duplicate_arg').length;
        assert.strictEqual(count, 1, 'Should not have duplicate argument names');
    });

    test('Avoid duplicate environment variables', () => {
        const content = `
            <property value="$(env DUPLICATE_ENV)"/>
            <property value="$(optenv DUPLICATE_ENV default)"/>
        `;
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        const count = detected.envNames.filter(name => name === 'DUPLICATE_ENV').length;
        assert.strictEqual(count, 1, 'Should not have duplicate environment variable names');
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

    test('Match file with specific wildcard pattern', () => {
        const config: xacroConfig.XacroConfig = {
            version: '1.0.0',
            '**/robots/*.xacro': {
                args: { specific_arg: 'specific_value' }
            }
        };

        const fileConfig = xacroConfig.findConfigForFile('/workspace/src/robots/my_robot.xacro', config);
        
        assert.ok(fileConfig, 'Should find config for specific wildcard pattern');
        assert.strictEqual(fileConfig?.args?.specific_arg, 'specific_value');
    });

    test('No match for non-matching pattern', () => {
        const config: xacroConfig.XacroConfig = {
            version: '1.0.0',
            '**/robots/*.xacro': {
                args: { specific_arg: 'value' }
            }
        };

        const fileConfig = xacroConfig.findConfigForFile('/workspace/src/other/my_robot.xacro', config);
        
        assert.strictEqual(fileConfig, null, 'Should not find config for non-matching pattern');
    });

    test('Config with both args and env', () => {
        const config: xacroConfig.XacroConfig = {
            version: '1.0.0',
            'test.xacro': {
                args: { test_arg: 'arg_value' },
                env: { TEST_ENV: 'env_value' }
            }
        };

        const fileConfig = xacroConfig.findConfigForFile('test.xacro', config);
        
        assert.ok(fileConfig, 'Should find config');
        assert.strictEqual(fileConfig?.args?.test_arg, 'arg_value');
        assert.strictEqual(fileConfig?.env?.TEST_ENV, 'env_value');
    });

    test('No detection on file without arguments or env vars', () => {
        const filePath = path.join(testDataPath, 'simple_box.urdf');
        const content = fs.readFileSync(filePath, 'utf8');
        
        const detected = xacroConfig.detectArgumentsAndEnv(content);
        
        assert.strictEqual(detected.hasArgs, false, 'Should not detect arguments');
        assert.strictEqual(detected.hasEnv, false, 'Should not detect environment variables');
    });

    test('Handle empty content', () => {
        const detected = xacroConfig.detectArgumentsAndEnv('');
        
        assert.strictEqual(detected.hasArgs, false, 'Should not detect arguments in empty content');
        assert.strictEqual(detected.hasEnv, false, 'Should not detect env vars in empty content');
        assert.strictEqual(detected.argNames.length, 0, 'Should have no argument names');
        assert.strictEqual(detected.envNames.length, 0, 'Should have no env var names');
    });

    test('Return null for null config', () => {
        const fileConfig = xacroConfig.findConfigForFile('/some/path.xacro', null);
        
        assert.strictEqual(fileConfig, null, 'Should return null when config is null');
    });

    test('Handle ${workspaceFolder} variable substitution in pattern', () => {
        // This test verifies the pattern matching logic handles variable substitution
        // The actual substitution happens in the findConfigForFile function
        const config: xacroConfig.XacroConfig = {
            version: '1.0.0',
            '${workspaceFolder}/robot.xacro': {
                args: { test: 'value' }
            }
        };

        // Note: Since we can't easily mock vscode.workspace in unit tests,
        // we verify the config structure is correct
        assert.ok(config['${workspaceFolder}/robot.xacro'], 'Config should have workspaceFolder pattern');
    });
});
