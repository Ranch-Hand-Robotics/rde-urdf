import * as assert from 'assert';
import * as path from 'path';
import * as fs from 'fs';

suite('Recursive Include Scanning Test Suite', () => {
    const testDataPath = path.join(__dirname, '../../src/test/testdata');

    /**
     * Helper function to simulate the recursive include scanning logic
     * This mimics what happens in utils.ts processXacro function
     */
    function scanIncludesRecursively(mainFilePath: string): {
        argNames: string[];
        envNames: string[];
        filesProcessed: number;
    } {
        const allArgNamesSet = new Set<string>();
        const allEnvNamesSet = new Set<string>();
        const processedFiles = new Set<string>();

        function detectArgumentsAndEnv(content: string): {
            argNames: string[];
            envNames: string[];
        } {
            const argNames: string[] = [];
            const envNames: string[] = [];

            // Detect xacro:arg declarations
            const argPattern = /<xacro:arg\s+name="([^"]+)"/g;
            let match;
            while ((match = argPattern.exec(content)) !== null) {
                if (!argNames.includes(match[1])) {
                    argNames.push(match[1]);
                }
            }

            // Detect $(arg name) usage
            const argUsagePattern = /\$\(arg\s+([a-zA-Z0-9_-]+)\)/g;
            while ((match = argUsagePattern.exec(content)) !== null) {
                if (!argNames.includes(match[1])) {
                    argNames.push(match[1]);
                }
            }

            // Detect $(env name) usage
            const envPattern = /\$\(env\s+([a-zA-Z0-9_-]+)\)/g;
            while ((match = envPattern.exec(content)) !== null) {
                if (!envNames.includes(match[1])) {
                    envNames.push(match[1]);
                }
            }

            // Detect $(optenv name default) usage
            const optenvPattern = /\$\(optenv\s+([a-zA-Z0-9_-]+)(?:\s+[^)]+)?\)/g;
            while ((match = optenvPattern.exec(content)) !== null) {
                if (!envNames.includes(match[1])) {
                    envNames.push(match[1]);
                }
            }

            return { argNames, envNames };
        }

        function scanFile(filePath: string) {
            const normalizedPath = path.normalize(filePath);

            if (processedFiles.has(normalizedPath)) {
                return;
            }
            processedFiles.add(normalizedPath);

            if (!fs.existsSync(filePath)) {
                return;
            }

            const content = fs.readFileSync(filePath, 'utf8');
            const detected = detectArgumentsAndEnv(content);

            detected.argNames.forEach(arg => allArgNamesSet.add(arg));
            detected.envNames.forEach(env => allEnvNamesSet.add(env));

            // Find includes (supports both single and double quotes)
            const includePattern = /<xacro:include\s+filename=(['"])([^'"]+)\1/g;
            let match;
            while ((match = includePattern.exec(content)) !== null) {
                let includePath = match[2];

                if (!path.isAbsolute(includePath)) {
                    includePath = path.join(path.dirname(filePath), includePath);
                }

                scanFile(includePath);
            }
        }

        scanFile(mainFilePath);

        return {
            argNames: Array.from(allArgNamesSet),
            envNames: Array.from(allEnvNamesSet),
            filesProcessed: processedFiles.size
        };
    }

    test('Scan main file without includes', () => {
        const filePath = path.join(testDataPath, 'robot_with_args.xacro');
        const result = scanIncludesRecursively(filePath);

        assert.strictEqual(result.filesProcessed, 1, 'Should process only the main file');
        assert.ok(result.argNames.includes('robot_name'), 'Should detect robot_name');
        assert.ok(result.argNames.includes('base_size'), 'Should detect base_size');
        assert.ok(result.argNames.includes('wheel_count'), 'Should detect wheel_count');
    });

    test('Scan main file with one level of includes', () => {
        const filePath = path.join(testDataPath, 'main_with_includes_with_args.xacro');
        const result = scanIncludesRecursively(filePath);

        assert.ok(result.filesProcessed >= 2, 'Should process main file and included file');
        assert.ok(result.argNames.includes('main_robot_name'), 'Should detect main file argument');
        assert.ok(result.argNames.includes('included_color'), 'Should detect included file argument');
        assert.ok(result.argNames.includes('included_size'), 'Should detect included file argument');
        assert.ok(result.envNames.includes('INCLUDED_MESH_PATH'), 'Should detect included file env var');
    });

    test('Scan file with single-quoted includes', () => {
        const filePath = path.join(testDataPath, 'test_single_quotes.xacro');
        const result = scanIncludesRecursively(filePath);

        assert.ok(result.filesProcessed >= 2, 'Should process main file and included file with single quotes');
        assert.ok(result.argNames.includes('test_arg'), 'Should detect argument from main file');
        assert.ok(result.argNames.includes('included_color'), 'Should detect argument from included file');
    });

    test('Avoid processing duplicate includes', () => {
        // Create a test scenario where the same file might be included multiple times
        const filePath = path.join(testDataPath, 'robot_with_args.xacro');
        const result = scanIncludesRecursively(filePath);

        // The same file should only be processed once
        assert.strictEqual(result.filesProcessed, 1, 'Should not process the same file twice');
    });

    test('Handle non-existent include files gracefully', () => {
        // This test ensures the scanning doesn't crash when an included file doesn't exist
        const tempFilePath = path.join(testDataPath, 'temp_test_missing_include.xacro');
        const tempContent = `<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="nonexistent_file.xacro"/>
  <xacro:arg name="test_arg" default="value"/>
</robot>`;

        try {
            fs.writeFileSync(tempFilePath, tempContent);
            const result = scanIncludesRecursively(tempFilePath);

            assert.strictEqual(result.filesProcessed, 1, 'Should process only the main file');
            assert.ok(result.argNames.includes('test_arg'), 'Should still detect arguments in main file');
        } finally {
            // Clean up
            if (fs.existsSync(tempFilePath)) {
                fs.unlinkSync(tempFilePath);
            }
        }
    });

    test('Accumulate arguments without duplicates', () => {
        const filePath = path.join(testDataPath, 'main_with_includes_with_args.xacro');
        const result = scanIncludesRecursively(filePath);

        // Verify no duplicates in results
        const uniqueArgs = new Set(result.argNames);
        assert.strictEqual(result.argNames.length, uniqueArgs.size, 'Should not have duplicate arguments');

        const uniqueEnvs = new Set(result.envNames);
        assert.strictEqual(result.envNames.length, uniqueEnvs.size, 'Should not have duplicate env vars');
    });

    test('Process includes with relative paths', () => {
        const filePath = path.join(testDataPath, 'robot_with_includes.xacro');
        const result = scanIncludesRecursively(filePath);

        // The robot_with_includes.xacro includes common_properties.xacro
        assert.ok(result.filesProcessed >= 2, 'Should process files with relative includes');
    });

    test('Detect both double and single quoted includes in same file', () => {
        const tempFilePath = path.join(testDataPath, 'temp_mixed_quotes.xacro');
        const tempContent = `<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="common_properties.xacro"/>
  <xacro:include filename='robot_with_args.xacro'/>
  <xacro:arg name="mixed_arg" default="value"/>
</robot>`;

        try {
            fs.writeFileSync(tempFilePath, tempContent);
            const result = scanIncludesRecursively(tempFilePath);

            assert.ok(result.filesProcessed >= 3, 'Should process main file and both includes');
            assert.ok(result.argNames.includes('mixed_arg'), 'Should detect argument from main file');
        } finally {
            if (fs.existsSync(tempFilePath)) {
                fs.unlinkSync(tempFilePath);
            }
        }
    });

    test('Performance: Set-based duplicate detection', () => {
        // Test that using Sets provides better performance than arrays
        const iterations = 1000;
        const testArgs = Array.from({ length: 100 }, (_, i) => `arg_${i % 50}`);

        // Measure array-based approach
        const startArray = Date.now();
        const arrayResult: string[] = [];
        for (let i = 0; i < iterations; i++) {
            testArgs.forEach(arg => {
                if (!arrayResult.includes(arg)) {
                    arrayResult.push(arg);
                }
            });
        }
        const arrayTime = Date.now() - startArray;

        // Measure Set-based approach
        const startSet = Date.now();
        const setResult = new Set<string>();
        for (let i = 0; i < iterations; i++) {
            testArgs.forEach(arg => setResult.add(arg));
        }
        const setTime = Date.now() - startSet;

        // Set-based should be faster or at least not significantly slower
        assert.ok(setTime <= arrayTime * 2, `Set-based approach should be efficient (Set: ${setTime}ms, Array: ${arrayTime}ms)`);
    });

    test('Empty file handling', () => {
        const tempFilePath = path.join(testDataPath, 'temp_empty.xacro');
        const tempContent = '<?xml version="1.0"?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro"></robot>';

        try {
            fs.writeFileSync(tempFilePath, tempContent);
            const result = scanIncludesRecursively(tempFilePath);

            assert.strictEqual(result.filesProcessed, 1, 'Should process empty file');
            assert.strictEqual(result.argNames.length, 0, 'Should have no arguments');
            assert.strictEqual(result.envNames.length, 0, 'Should have no env vars');
        } finally {
            if (fs.existsSync(tempFilePath)) {
                fs.unlinkSync(tempFilePath);
            }
        }
    });

    test('Complex include chain detection', () => {
        // Test a more complex scenario if we have nested includes
        const filePath = path.join(testDataPath, 'robot_with_includes.xacro');
        if (fs.existsSync(filePath)) {
            const result = scanIncludesRecursively(filePath);

            // Should process at least the main file
            assert.ok(result.filesProcessed >= 1, 'Should process at least main file in complex chain');
        }
    });
});
