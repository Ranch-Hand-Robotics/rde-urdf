const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');
const ts = require('typescript');
const { Client } = require('@modelcontextprotocol/sdk/client/index.js');
const { StreamableHTTPClientTransport } = require('@modelcontextprotocol/sdk/client/streamableHttp.js');

class EventEmitter {
  listeners = new Set();
  event = listener => {
    this.listeners.add(listener);
    return { dispose: () => this.listeners.delete(listener) };
  };
  fire() { for (const listener of this.listeners) { listener(); } }
  dispose() { this.listeners.clear(); }
}

// Each module cache represents a separate extension host. Only the VS Code UI
// and renderer are mocked; discovery, HTTP, credentials, and the MCP SDK are real.
function createWindow(t, name = 'same-workspace') {
  const opened = [];
  const logs = [];
  const clients = [];
  const registrations = [];
  const commands = new Map();
  const foldersChanged = new EventEmitter();
  const trace = { appendLine: line => logs.push(line) };
  const manager = {
    getExistingPreview: () => undefined,
    preview(uri) {
      opened.push(uri.fsPath);
      this.activePreview = { takeScreenshot: async () => Buffer.from(name).toString('base64') };
    }
  };
  const vscode = {
    EventEmitter,
    McpHttpServerDefinition: class {
      constructor(label, uri, headers, version) { Object.assign(this, { label, uri, headers, version }); }
    },
    Uri: {
      parse: uri => new URL(uri),
      file: fsPath => ({ fsPath, toString: () => fsPath })
    },
    workspace: {
      name,
      workspaceFolders: [{ uri: { toString: () => path.join(__dirname, name), fsPath: path.join(__dirname, name) } }],
      onDidChangeWorkspaceFolders: foldersChanged.event,
      fs: { stat: async () => ({}) }
    },
    lm: {
      registerMcpServerDefinitionProvider(id, provider) {
        registrations.push({ id, provider });
        return { dispose() {} };
      }
    },
    commands: {
      registerCommand(id, callback) {
        commands.set(id, callback);
        return { dispose: () => commands.delete(id) };
      }
    }
  };
  const mocks = {
    vscode,
    './extension': { tracing: trace, urdfManager: manager },
    './openscad': {}
  };
  const cache = new Map();
  function load(name) {
    if (Object.hasOwn(mocks, name)) { return mocks[name]; }
    if (!name.startsWith('./')) { return require(name); }
    if (cache.has(name)) { return cache.get(name); }
    const filename = path.join(__dirname, 'src', `${name.slice(2)}.ts`);
    const { outputText } = ts.transpileModule(fs.readFileSync(filename, 'utf8'), {
      compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 }
    });
    const module = { exports: {} };
    cache.set(name, module.exports);
    vm.runInThisContext(`(function(require, module, exports) { ${outputText}\n})`, { filename })(load, module, module.exports);
    return module.exports;
  }
  const { WindowMcpConnection } = load('./mcpConnection');
  const controller = new WindowMcpConnection(trace);
  const context = { subscriptions: [] };
  controller.register(context);
  t.after(async () => {
    for (const client of clients) { await client.close(); }
    await controller.dispose();
    for (const disposable of context.subscriptions) { disposable.dispose(); }
  });
  async function connect(definition, resolve = true) {
    definition ??= (await controller.provideMcpServerDefinitions())[0];
    if (resolve) { definition = await controller.resolveMcpServerDefinition(definition); }
    const transport = new StreamableHTTPClientTransport(definition.uri, {
      requestInit: { headers: definition.headers }
    });
    const client = new Client({ name: 'window-isolation-test', version: '1.0.0' });
    clients.push(client);
    await client.connect(transport);
    return { client, transport };
  }
  return { controller, connect, opened, logs, registrations, commands, vscode, foldersChanged, ...load('./mcp') };
}

async function request(definition, method = 'POST', headers = definition.headers, body) {
  const response = await fetch(definition.uri, {
    method,
    headers: { 'Content-Type': 'application/json', Accept: 'application/json, text/event-stream', ...headers },
    body: method === 'POST' ? JSON.stringify(body ?? {
      jsonrpc: '2.0', id: 1, method: 'initialize',
      params: { protocolVersion: '2025-03-26', capabilities: {}, clientInfo: { name: 'test', version: '1' } }
    }) : undefined,
    signal: AbortSignal.timeout(5000)
  });
  const text = await response.text();
  return { status: response.status, text };
}

for (const sameWorkspace of [true, false]) {
  test(`previews stay in their window (${sameWorkspace ? 'same' : 'different'} workspace)`, { timeout: 15000 }, async t => {
    const a = createWindow(t);
    const b = createWindow(t, sameWorkspace ? 'same-workspace' : 'other-workspace');
    const [[ad], [bd]] = await Promise.all([
      a.controller.provideMcpServerDefinitions(), b.controller.provideMcpServerDefinitions()
    ]);
    assert.notEqual(ad.uri.toString(), bd.uri.toString());
    assert.equal(ad.label, 'URDF', 'Preserve bundled agents\' urdf/* tool references');
    assert.notEqual(ad.headers['x-urdf-instance'], bd.headers['x-urdf-instance']);
    assert.equal(ad.headers.Authorization, undefined, 'Do not cache credentials in discovery');
    assert.equal(ad.uri.hostname, '127.0.0.1');
    const [{ client: ac }, { client: bc }] = await Promise.all([a.connect(ad), b.connect(bd)]);
    const filename = path.join(__dirname, 'src/test/testdata/simple_cube.scad');
    await ac.callTool({ name: 'take_screenshot', arguments: { filename } });
    assert.deepEqual(a.opened, [filename]);
    assert.deepEqual(b.opened, []);
    await bc.callTool({ name: 'take_screenshot_of_file', arguments: { filename } });
    assert.deepEqual(b.opened, [filename]);
    await ac.callTool({ name: 'take_screenshot', arguments: {} });
    assert.deepEqual(a.opened, [filename]);
    const resolved = await a.controller.resolveMcpServerDefinition(ad);
    await a.controller.dispose();
    assert.ok((await bc.listTools()).tools.length > 0, 'Closing A must not stop B');
    assert.ok(a.logs.some(line => line.includes('workspace:')));
    assert.ok(!a.logs.join('\n').includes(resolved.headers.Authorization.slice(7)), 'Never log credentials');
  });
}

test('missing, wrong-window, and stale credentials fail before tool dispatch on every HTTP method', async t => {
  const a = createWindow(t);
  const b = createWindow(t);
  const ad = await a.controller.resolveMcpServerDefinition((await a.controller.provideMcpServerDefinitions())[0]);
  const bd = await b.controller.resolveMcpServerDefinition((await b.controller.provideMcpServerDefinitions())[0]);
  const { transport } = await a.connect(ad);
  for (const method of ['POST', 'GET', 'DELETE']) {
    for (const credentials of [{}, bd.headers, { ...ad.headers, Authorization: 'Bearer bad' },
      { ...ad.headers, 'x-urdf-instance': bd.headers['x-urdf-instance'] }]) {
      const result = await request(ad, method, { ...credentials, 'mcp-session-id': transport.sessionId }, {
        jsonrpc: '2.0', id: 2, method: 'tools/call',
        params: { name: 'take_screenshot', arguments: { filename: 'robot.urdf' } }
      });
      assert.equal(result.status, 403);
    }
  }
  assert.equal((await request(ad, 'POST', {})).status, 403, 'Initialization must also require credentials');
  assert.deepEqual(a.opened, []);
  assert.deepEqual(b.opened, []);
  await assert.rejects(a.controller.resolveMcpServerDefinition(bd), /another VS Code window/);
  const resolved = await a.controller.resolveMcpServerDefinition({ ...ad, uri: bd.uri });
  assert.equal(resolved.uri.toString(), ad.uri.toString(), 'Never trust a cached or replaced endpoint');
  assert.equal((await request(bd, 'POST', { ...bd.headers, 'mcp-session-id': transport.sessionId })).status, 400,
    'Another window\'s protocol session is not valid even with current credentials');
});

test('multiple clients keep independent transports in the same window', async t => {
  const window = createWindow(t);
  const first = await window.connect();
  const second = await window.connect();
  assert.notEqual(first.transport.sessionId, second.transport.sessionId);
  assert.ok((await first.client.listTools()).tools.length > 0);
  assert.ok((await second.client.listTools()).tools.length > 0);
  await first.transport.terminateSession();
  await first.client.close();
  assert.ok((await second.client.listTools()).tools.length > 0);
  const third = await window.connect();
  assert.ok((await third.client.listTools()).tools.length > 0);
});

test('provider registration stays singular and definitions refresh after stop/restart', async t => {
  const window = createWindow(t);
  let changes = 0;
  window.controller.onDidChangeMcpServerDefinitions(() => changes++);
  const results = await Promise.all(Array.from({ length: 5 }, () => window.controller.provideMcpServerDefinitions()));
  const old = results[0][0];
  const oldConnection = await window.controller.resolveMcpServerDefinition(old);
  assert.ok(results.every(([definition]) => definition.uri.toString() === old.uri.toString()));
  assert.equal(window.logs.filter(line => line.includes('listening at')).length, 1);
  await window.commands.get('urdf-editor.stopMcpServer')();
  assert.deepEqual(await window.controller.provideMcpServerDefinitions(), []);
  await assert.rejects(window.controller.resolveMcpServerDefinition(old), /stopped/);
  await window.commands.get('urdf-editor.startMcpServer')();
  const current = await window.controller.resolveMcpServerDefinition(old);
  assert.equal(current.label, old.label);
  assert.notEqual(current.version, old.version);
  assert.notEqual(current.headers.Authorization, oldConnection.headers.Authorization);
  assert.equal((await request(current, 'POST', oldConnection.headers)).status, 403);
  assert.ok((await (await window.connect(current)).client.listTools()).tools.length > 0);
  assert.equal(window.registrations.length, 1);
  assert.ok(changes >= 3);
});

test('reusing a closed listener port never accepts its previous credentials', async t => {
  const window = createWindow(t);
  const old = await window.controller.resolveMcpServerDefinition((await window.controller.provideMcpServerDefinitions())[0]);
  await window.controller.stop();
  const replacement = new window.UrdfMcpServer(Number(old.uri.port));
  t.after(() => replacement.stop());
  await replacement.start();
  assert.equal(replacement.getConnection().url, old.uri.toString());
  assert.equal((await request(old)).status, 403);
  const connection = replacement.getConnection();
  const definition = { uri: new URL(connection.url), headers: connection.headers };
  assert.ok((await (await window.connect(definition, false)).client.listTools()).tools.length > 0);
});

test('empty and multi-root windows work, folder changes refresh discovery without renaming tools', async t => {
  const window = createWindow(t, '');
  window.vscode.workspace.workspaceFolders = undefined;
  const [empty] = await window.controller.provideMcpServerDefinitions();
  assert.equal(empty.label, 'URDF');
  window.vscode.workspace.name = 'two-roots';
  window.vscode.workspace.workspaceFolders = ['a', 'b'].map(name => ({ uri: { toString: () => name } }));
  let changes = 0;
  window.controller.onDidChangeMcpServerDefinitions(() => changes++);
  window.foldersChanged.fire();
  const [multi] = await window.controller.provideMcpServerDefinitions();
  assert.equal(multi.label, 'URDF');
  assert.equal(multi.uri.toString(), empty.uri.toString());
  assert.equal(changes, 1);
});

test('dispose during startup leaves no discoverable listener', async t => {
  const window = createWindow(t);
  const start = window.controller.start();
  const shutdown = window.controller.dispose();
  await Promise.all([start, shutdown]);
  await window.controller.start();
  assert.deepEqual(await window.controller.provideMcpServerDefinitions(), []);
  assert.equal(window.logs.filter(line => line.includes('listening at')).length, 0);
});

test('a failed start does not poison the lifecycle queue', async t => {
  const window = createWindow(t);
  const start = window.UrdfMcpServer.prototype.start;
  let attempts = 0;
  window.UrdfMcpServer.prototype.start = async function () {
    if (++attempts === 1) { throw new Error('Simulated listener failure'); }
    return start.call(this);
  };
  await assert.rejects(window.controller.start(), /Simulated listener failure/);
  const [definition] = await window.controller.provideMcpServerDefinitions();
  assert.ok((await (await window.connect(definition)).client.listTools()).tools.length > 0);
  assert.equal(attempts, 2);
});

test('shutdown waits for an in-flight start and closes the resulting listener', async t => {
  const window = createWindow(t);
  const start = window.UrdfMcpServer.prototype.start;
  let listening;
  let release;
  let listener;
  const ready = new Promise(resolve => { listening = resolve; });
  const gate = new Promise(resolve => { release = resolve; });
  window.UrdfMcpServer.prototype.start = async function () {
    await start.call(this);
    listener = this;
    listening();
    await gate;
  };
  const starting = window.controller.start();
  await ready;
  const shutdown = window.controller.dispose();
  const discovering = window.controller.provideMcpServerDefinitions();
  release();
  await Promise.all([starting, shutdown]);
  assert.deepEqual(await discovering, []);
  assert.equal(listener.getStatus().isRunning, false);
  assert.throws(() => listener.getConnection(), /not running/);
});