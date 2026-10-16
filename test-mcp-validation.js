#!/usr/bin/env node
/**
 * Compatibility entry point for the isolated MCP regression suite.
 * Never probe a fixed port: it may belong to a different VS Code window.
 * The suite uses real SDK/HTTP connections with mocked VS Code and rendering;
 * it tests routing and validator parameters, not OpenSCAD compilation itself.
 */
const { spawnSync } = require('node:child_process');
const path = require('node:path');

const result = spawnSync(process.execPath, [
  '--test',
  path.join(__dirname, 'test-mcp-schema.js'),
  path.join(__dirname, 'test-mcp-isolation.js')
], { stdio: 'inherit' });
if (result.error) { console.error(result.error.message); }
process.exitCode = result.status ?? 1;
