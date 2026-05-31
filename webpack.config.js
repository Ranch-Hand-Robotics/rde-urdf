//@ts-check

'use strict';

const path = require('path');
const fs = require('fs');
const CopyWebpackPlugin = require('copy-webpack-plugin');

const localBabylonRosDistDir = path.resolve(__dirname, '..', 'babylon_ros', 'dist');
const packageBabylonRosDistDir = path.resolve(
  __dirname,
  'node_modules',
  '@ranchhandrobotics',
  'babylon_ros',
  'dist',
);
const babylonRosDistDir = fs.existsSync(localBabylonRosDistDir)
  ? localBabylonRosDistDir
  : packageBabylonRosDistDir;
const openscadCopyCandidates = [
  // New/actual babylon_ros layout (built from local fork or downloaded collateral)
  path.join(babylonRosDistDir, 'openscad-wasm-build', 'dist'),
  // Back-compat layout used by some older package variants
  path.join(babylonRosDistDir, 'openscad-wasm'),
];
const openscadCopySource = openscadCopyCandidates.find((candidate) => fs.existsSync(path.join(candidate, 'openscad.wasm')));
const openscadNodeWorkerSource = path.join(babylonRosDistDir, 'workers', 'openscadWorker.node.js');

//@ts-check
/** @typedef {import('webpack').Configuration} WebpackConfig **/

/** @type WebpackConfig */
const extensionConfig = {
  target: 'node', // VS Code extensions run in a Node.js-context 📖 -> https://webpack.js.org/configuration/node/
	mode: 'none', // this leaves the source code as close as possible to the original (when packaging we set this to 'production')

  entry: './src/extension.ts', // the entry point of this extension, 📖 -> https://webpack.js.org/configuration/entry-context/
  output: {
    // the bundle is stored in the 'dist' folder (check package.json), 📖 -> https://webpack.js.org/configuration/output/
    path: path.resolve(__dirname, 'dist'),
    filename: 'extension.js',
    libraryTarget: 'commonjs2'
  },
  externals: {
    vscode: 'commonjs vscode' // the vscode-module is created on-the-fly and must be excluded. Add other modules that cannot be webpack'ed, 📖 -> https://webpack.js.org/configuration/externals/
    // modules added here also need to be added in the .vscodeignore file
  },
  resolve: {
    // support reading TypeScript and JavaScript files, 📖 -> https://github.com/TypeStrong/ts-loader
    extensions: ['.ts', '.js'],
    alias: {
      'handlebars' : 'handlebars/dist/handlebars.js',
      '@ranchhandrobotics/babylon_ros/openscad': path.join(babylonRosDistDir, 'openscad.js'),
    }
  },
  module: {
    rules: [
      {
        test: /\.ts$/,
        exclude: /node_modules/,
        use: [
          {
            loader: 'ts-loader'
          }
        ]
      }
    ]
  },
  devtool: 'source-map',
  infrastructureLogging: {
    level: "log", // enables logging required for problem matchers
  },
  plugins: [
    new CopyWebpackPlugin({
      patterns: [
        { from: 'snippets', to: 'snippets' },
        { from: 'schemas', to: 'schemas' },
        { from: '.github', to: '.github' },
        { from: 'assets', to: 'assets' },
        { from: 'syntaxes', to: 'syntaxes' },
        { from: 'language-configuration.json', to: 'language-configuration.json' },
        { from: 'CHANGELOG.md', to: 'CHANGELOG.md' },
        { from: 'README.md', to: 'README.md' },
        // Copy openscad-wasm runtime files from babylon_ros so MCP/preview tools use the same local build.
        ...(openscadCopySource
          ? [{ from: openscadCopySource, to: 'openscad-wasm' }]
          : [{
              from: path.resolve(__dirname, 'node_modules', '@ranchhandrobotics', 'babylon_ros', 'dist', 'openscad-wasm'),
              to: 'openscad-wasm',
              noErrorOnMissing: true,
            }]),
        ...(fs.existsSync(openscadNodeWorkerSource)
          ? [{ from: openscadNodeWorkerSource, to: 'workers/openscadWorker.node.js' }]
          : []),
        // If you need to copy any other assets, add them here
      ],
    }),
  ],
};

const webviewConfig = {
  target:  ["web", "es2022"], // VS Code extensions run in a Node.js-context 📖 -> https://webpack.js.org/configuration/node/
  mode: 'none', // this leaves the source code as close as possible to the original (when packaging we set this to 'production')

  entry: './src/webview/webview.ts', // the entry point of this extension, 📖 -> https://webpack.js.org/configuration/entry-context/
  experiments: { outputModule: true, topLevelAwait: true },
  output: {
    // the bundle is stored in the 'dist' folder (check package.json), 📖 -> https://webpack.js.org/configuration/output/
    path: path.resolve(__dirname, 'dist'),
    filename: 'webview.js',
    libraryTarget: 'module',
    chunkFormat: "module"
  },
  externals: {
    vscode: 'commonjs vscode' // the vscode-module is created on-the-fly and must be excluded. Add other modules that cannot be webpack'ed, 📖 -> https://webpack.js.org/configuration/externals/
    // modules added here also need to be added in the .vscodeignore file
  },
  resolve: {
    extensions: ['.ts', '.js', '.mts', '.css']
  },
  module: {
    rules: [
      {
        test: /\.ts$/,
        exclude: /node_modules/,
        use: [
          {
            loader: 'ts-loader'
          }
        ]
      }
    ]
  },
  devtool: 'source-map',
  infrastructureLogging: {
    level: "log", // enables logging required for problem matchers
  },
  plugins: [
    new CopyWebpackPlugin({
      patterns: [
        { from: 'snippets', to: 'snippets' },
        { from: 'schemas', to: 'schemas' },
        { from: '.github', to: '.github' },
        { from: 'assets', to: 'assets' },
        { from: 'syntaxes', to: 'syntaxes' },
        { from: 'language-configuration.json', to: 'language-configuration.json' },
        { from: 'CHANGELOG.md', to: 'CHANGELOG.md' },
        { from: 'README.md', to: 'README.md' },
        // If you need to copy any other assets, add them here
      ],
    }),
  ],
};

module.exports = [ extensionConfig, webviewConfig ];