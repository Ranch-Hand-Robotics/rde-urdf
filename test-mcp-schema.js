const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');
const ts = require('typescript');
const { Client } = require('@modelcontextprotocol/sdk/client/index.js');
const { InMemoryTransport } = require('@modelcontextprotocol/sdk/inMemory.js');

// Exercise the actual tool registration and SDK schema conversion without a VS Code host.
async function createClient(t) {
  const validations = [];
  const fileChecks = [];
  const mocks = {
    vscode: {
      Uri: { file: fsPath => ({ fsPath }) },
      workspace: {
        workspaceFolders: [{ uri: { fsPath: __dirname } }],
        fs: { stat: async uri => { fileChecks.push(uri.fsPath); } }
      }
    },
    './extension': { tracing: { appendLine() {} } },
    './openscad': {
      validateOpenSCAD: async (...args) => {
        validations.push(args);
        return { valid: true, errors: [], warnings: [] };
      }
    }
  };
  const filename = path.join(__dirname, 'src', 'mcp.ts');
  const { outputText } = ts.transpileModule(fs.readFileSync(filename, 'utf8'), {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 }
  });
  const module = { exports: {} };
  const load = vm.runInThisContext(
    `(function(require, module, exports) { ${outputText}\n})`, { filename }
  );
  load(name => Object.hasOwn(mocks, name) ? mocks[name] : require(name), module, module.exports);

  const server = new module.exports.UrdfMcpServer().createServer();
  const client = new Client({ name: 'schema-regression-test', version: '1.0.0' });
  t.after(async () => {
    await client.close();
    await server.close();
  });
  const [clientTransport, serverTransport] = InMemoryTransport.createLinkedPair();
  await server.connect(serverTransport);
  await client.connect(clientTransport);
  return { client, validations, fileChecks };
}

test('validate_openscad advertises required filename and optional content', async t => {
  const { client } = await createClient(t);
  const { tools } = await client.listTools();
  const tool = tools.find(tool => tool.name === 'validate_openscad');
  assert.ok(tool, 'Validator must be discoverable');
  assert.equal(tool.inputSchema.properties?.filename?.type, 'string');
  assert.equal(tool.inputSchema.properties?.content?.type, 'string');
  assert.deepEqual(tool.inputSchema.required, ['filename']);
});

test('validate_openscad rejects missing or incorrectly typed arguments', async t => {
  const { client, validations } = await createClient(t);
  for (const args of [{}, { filename: 123 }, { filename: 'test.scad', content: 123 }]) {
    const result = await client.callTool({ name: 'validate_openscad', arguments: args });
    assert.equal(result.isError, true, `Expected invalid arguments: ${JSON.stringify(args)}`);
  }
  assert.equal(validations.length, 0);
});

test('validate_openscad forwards an absolute filename', async t => {
  const { client, validations, fileChecks } = await createClient(t);
  const filename = path.join(__dirname, 'src', 'test', 'testdata', 'simple_cube.scad');
  const result = await client.callTool({ name: 'validate_openscad', arguments: { filename } });
  assert.match(result.content[0].text, /compiles successfully/);
  assert.deepEqual(validations, [[filename, undefined, __dirname]]);
  assert.deepEqual(fileChecks, [filename]);
});

test('validate_openscad resolves workspace-relative filenames', async t => {
  const { client, validations } = await createClient(t);
  const filename = 'src/test/testdata/simple_cube.scad';
  await client.callTool({ name: 'validate_openscad', arguments: { filename } });
  assert.deepEqual(validations, [[path.join(__dirname, filename), undefined, __dirname]]);
});

test('validate_openscad forwards inline content without checking disk', async t => {
  const { client, validations, fileChecks } = await createClient(t);
  const filename = path.join(__dirname, 'unsaved.scad');
  const content = 'cube([10, 10, 10]);';
  const result = await client.callTool({ name: 'validate_openscad', arguments: { filename, content } });
  assert.match(result.content[0].text, /compiles successfully/);
  assert.deepEqual(validations, [[filename, content, __dirname]]);
  assert.deepEqual(fileChecks, []);
});