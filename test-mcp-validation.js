#!/usr/bin/env node
/**
 * Simple test script to verify the validate_openscad MCP endpoint
 * This script simulates what an MCP client would do to call the validation endpoint
 */

const http = require('http');

// MCP server configuration
const MCP_PORT = 3005;
const MCP_HOST = 'localhost';

// Test file paths
const validFile = __dirname + '/src/test/testdata/simple_cube.scad';
const invalidFile = __dirname + '/src/test/testdata/invalid_syntax.scad';

/**
 * Make an MCP request
 */
async function mcpRequest(method, params, sessionId = null) {
  const requestBody = {
    jsonrpc: '2.0',
    id: Date.now(),
    method: method,
    params: params
  };

  const headers = {
    'Content-Type': 'application/json',
  };

  if (sessionId) {
    headers['mcp-session-id'] = sessionId;
  }

  return new Promise((resolve, reject) => {
    const options = {
      hostname: MCP_HOST,
      port: MCP_PORT,
      path: '/mcp',
      method: 'POST',
      headers: headers
    };

    const req = http.request(options, (res) => {
      let data = '';

      res.on('data', (chunk) => {
        data += chunk;
      });

      res.on('end', () => {
        try {
          const response = JSON.parse(data);
          resolve(response);
        } catch (err) {
          reject(new Error(`Failed to parse response: ${err.message}`));
        }
      });
    });

    req.on('error', (err) => {
      reject(err);
    });

    req.write(JSON.stringify(requestBody));
    req.end();
  });
}

/**
 * Test the validate_openscad endpoint
 */
async function testValidation() {
  console.log('Testing MCP validate_openscad endpoint...\n');

  try {
    // Initialize session
    console.log('1. Initializing MCP session...');
    const initResponse = await mcpRequest('initialize', {
      protocolVersion: '2024-11-05',
      capabilities: {},
      clientInfo: {
        name: 'test-client',
        version: '1.0.0'
      }
    });
    
    const sessionId = initResponse.result?.serverInfo?.sessionId || 'test-session';
    console.log(`   Session ID: ${sessionId}\n`);

    // Test valid file
    console.log('2. Testing with VALID OpenSCAD file...');
    console.log(`   File: ${validFile}`);
    const validResponse = await mcpRequest('tools/call', {
      name: 'validate_openscad',
      arguments: {
        filename: validFile
      }
    }, sessionId);
    
    console.log('   Response:', JSON.stringify(validResponse.result, null, 2));
    console.log('');

    // Test invalid file
    console.log('3. Testing with INVALID OpenSCAD file...');
    console.log(`   File: ${invalidFile}`);
    const invalidResponse = await mcpRequest('tools/call', {
      name: 'validate_openscad',
      arguments: {
        filename: invalidFile
      }
    }, sessionId);
    
    console.log('   Response:', JSON.stringify(invalidResponse.result, null, 2));
    console.log('');

    // Test with content
    console.log('4. Testing with inline content...');
    const testContent = 'cube([10, 10, 10]);';
    const contentResponse = await mcpRequest('tools/call', {
      name: 'validate_openscad',
      arguments: {
        filename: '/tmp/test.scad',
        content: testContent
      }
    }, sessionId);
    
    console.log('   Content:', testContent);
    console.log('   Response:', JSON.stringify(contentResponse.result, null, 2));
    console.log('');

    console.log('✓ All tests completed successfully!');

  } catch (error) {
    console.error('✗ Test failed:', error.message);
    console.error('\nMake sure:');
    console.error('1. VS Code extension is installed and running');
    console.error('2. A URDF/Xacro/OpenSCAD preview is open (to start the MCP server)');
    console.error('3. MCP server is running on port 3005');
    process.exit(1);
  }
}

// Run tests
testValidation();
