# Unit Test Documentation for Xacro Configuration Feature

## Overview
This document describes the comprehensive unit tests added to verify the xacro argument and environment variable detection functionality.

## Test Files

### 1. `src/test/xacroConfig.test.ts` (22 tests)
Tests for the xacroConfig module that handles detection and configuration management.

#### Detection Tests
| Test Name | Description | Verifies |
|-----------|-------------|----------|
| Detect arguments in xacro file | Tests basic argument detection | Detects `robot_name`, `base_size`, `wheel_count` arguments |
| Detect environment variables in xacro file | Tests environment variable detection | Detects `ROBOT_COLOR`, `ROBOT_PREFIX` env vars |
| Detect both arguments and environment variables | Tests combined detection | Detects both args and env vars in same file |
| Detect arguments in included files | Tests detection in included files | Detects `included_color`, `included_size` from included file |
| Detect arguments from main file with includes | Tests main file detection | Detects `main_robot_name` from main file |

#### Pattern Detection Tests
| Test Name | Description | Verifies |
|-----------|-------------|----------|
| Detect include directives with double quotes | Tests double quote parsing | Matches `filename="test.xacro"` |
| Detect include directives with single quotes | Tests single quote parsing | Matches `filename='test.xacro'` |
| Detect xacro:arg declarations | Tests xacro:arg syntax | Detects `<xacro:arg name="test_arg"/>` |
| Detect $(arg name) usage | Tests arg substitution syntax | Detects `$(arg my_arg)` |
| Detect $(env name) usage | Tests env substitution syntax | Detects `$(env MY_ENV_VAR)` |
| Detect $(optenv name default) usage | Tests optenv syntax | Detects `$(optenv MY_OPT_ENV default)` |

#### Duplicate Avoidance Tests
| Test Name | Description | Verifies |
|-----------|-------------|----------|
| Avoid duplicate arguments | Tests deduplication | Same argument declared/used multiple times only counted once |
| Avoid duplicate environment variables | Tests env deduplication | Same env var used multiple times only counted once |

#### Configuration Matching Tests
| Test Name | Description | Verifies |
|-----------|-------------|----------|
| Match file with exact path | Tests exact path matching | `/path/to/robot.xacro` matches exactly |
| Match file with wildcard pattern | Tests wildcard matching | `**/*.xacro` matches any .xacro file |
| Match file with specific wildcard pattern | Tests specific wildcards | `**/robots/*.xacro` matches specific paths |
| No match for non-matching pattern | Tests negative case | Non-matching paths return null |
| Config with both args and env | Tests combined config | Config can have both args and env sections |

#### Edge Cases
| Test Name | Description | Verifies |
|-----------|-------------|----------|
| No detection on file without arguments or env vars | Tests negative case | Files without args/env return false |
| Handle empty content | Tests empty string | Empty content handled gracefully |
| Return null for null config | Tests null input | Null config returns null |
| Handle ${workspaceFolder} variable substitution in pattern | Tests variable handling | Config supports ${workspaceFolder} patterns |

### 2. `src/test/suite/recursiveInclude.test.ts` (11 tests)
Integration tests for the recursive include scanning feature in utils.ts.

#### Recursive Scanning Tests
| Test Name | Description | Verifies |
|-----------|-------------|----------|
| Scan main file without includes | Tests single file | Processes 1 file, detects all arguments |
| Scan main file with one level of includes | Tests 1-level recursion | Processes main + included file, accumulates all args |
| Scan file with single-quoted includes | Tests quote flexibility | Single quotes work same as double quotes |
| Avoid processing duplicate includes | Tests deduplication | Same file only processed once |
| Handle non-existent include files gracefully | Tests error handling | Missing includes don't crash, main file still processed |

#### Accumulation Tests
| Test Name | Description | Verifies |
|-----------|-------------|----------|
| Accumulate arguments without duplicates | Tests Set usage | No duplicate args in final result |
| Process includes with relative paths | Tests path resolution | Relative include paths work correctly |
| Detect both double and single quoted includes in same file | Tests mixed quotes | Both quote styles in one file work |

#### Performance Tests
| Test Name | Description | Verifies |
|-----------|-------------|----------|
| Performance: Set-based duplicate detection | Benchmarks performance | Set-based approach is efficient |
| Empty file handling | Tests edge case | Empty files don't cause errors |
| Complex include chain detection | Tests multi-level includes | Handles complex nested structures |

## Test Coverage Summary

### Lines of Test Code
- **xacroConfig.test.ts**: ~250 lines
- **recursiveInclude.test.ts**: ~330 lines
- **Total**: ~580 lines of test code

### Features Tested
✅ Argument detection (`xacro:arg`, `$(arg ...)`)
✅ Environment variable detection (`$(env ...)`, `$(optenv ...)`)
✅ Include directive parsing (single and double quotes)
✅ Recursive file scanning
✅ Duplicate prevention (args, env vars, files)
✅ Pattern matching (exact, wildcards, specific patterns)
✅ Path resolution (relative, absolute)
✅ Error handling (missing files, empty content, null inputs)
✅ Performance (Set vs Array for deduplication)
✅ Complex scenarios (nested includes, mixed quotes, combined configs)

## Running the Tests

### Compile Tests
```bash
npm run compile-tests
```

### Run All Tests
```bash
npm test
```

### Test Files Location
- Compiled tests: `out/src/test/xacroConfig.test.js`
- Compiled tests: `out/src/test/suite/recursiveInclude.test.js`
- Test data: `src/test/testdata/*.xacro`

## Test Data Files Used

### Created for This Feature
- `robot_with_args.xacro` - File with xacro:arg declarations
- `robot_with_env.xacro` - File with environment variables
- `robot_with_args_and_env.xacro` - File with both
- `included_with_args.xacro` - Include file with arguments
- `main_with_includes_with_args.xacro` - Main file including above
- `test_single_quotes.xacro` - Tests single quote support

### Existing Test Files Used
- `simple_box.urdf` - File without args/env (negative test)
- `robot_with_includes.xacro` - Complex include scenario
- `common_properties.xacro` - Shared properties file

## Expected Test Results

All 33 tests should pass:
- ✅ 22 tests in xacroConfig.test.ts
- ✅ 11 tests in recursiveInclude.test.ts

## Test Maintenance

### Adding New Tests
When adding new features:
1. Add detection tests to `xacroConfig.test.ts` for pattern detection
2. Add integration tests to `recursiveInclude.test.ts` for file processing
3. Create test data files in `src/test/testdata/` as needed
4. Update this documentation

### Test Philosophy
- **Unit tests**: Test individual functions in isolation
- **Integration tests**: Test the full scanning pipeline
- **Edge cases**: Test error conditions and boundary cases
- **Performance**: Verify efficiency of algorithms

## Notes

- Tests use the `assert` module from Node.js
- Tests follow the Mocha test framework structure (`suite`, `test`)
- Tests are TypeScript files that compile to JavaScript
- Pre-existing dependency errors in compilation don't affect test functionality
- Tests are self-contained and don't require VS Code workspace
