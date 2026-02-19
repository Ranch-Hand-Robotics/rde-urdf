import { fork } from 'child_process';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import * as fs from 'fs';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Test the worker with SVG export
const workerPath = join(__dirname, 'dist', 'workers', 'openscadWorker.js');
const testScadPath = join(__dirname, 'src', 'test', 'testdata', 'svg_test_shapes.scad');

console.log('Worker path:', workerPath);
console.log('Test SCAD path:', testScadPath);
console.log('Worker exists:', fs.existsSync(workerPath));
console.log('Test file exists:', fs.existsSync(testScadPath));

if (!fs.existsSync(workerPath)) {
  console.error('Worker not found! Need to compile first.');
  process.exit(1);
}

if (!fs.existsSync(testScadPath)) {
  console.error('Test file not found!');
  process.exit(1);
}

console.log('\n=== Testing SVG export via worker ===\n');

const worker = fork(workerPath, [], {
  stdio: ['pipe', 'pipe', 'pipe', 'ipc']
});

worker.on('message', (response) => {
  if (response.progress) {
    console.log('Progress:', response.progress);
  } else if (response.success) {
    console.log('\n✓ SUCCESS!');
    console.log('Output path:', response.outputPath || response.svgPath);
    
    // Read and display the SVG
    if (response.svgPath || response.outputPath) {
      const svgPath = response.svgPath || response.outputPath;
      if (fs.existsSync(svgPath)) {
        const svgContent = fs.readFileSync(svgPath, 'utf8');
        console.log('\nSVG content length:', svgContent.length);
        console.log('\nFirst 500 chars of SVG:');
        console.log(svgContent.substring(0, 500));
      }
    }
    process.exit(0);
  } else if (response.error) {
    console.error('\n✗ FAILED:', response.error);
    process.exit(1);
  }
});

worker.on('error', (error) => {
  console.error('Worker error:', error);
  process.exit(1);
});

worker.on('exit', (code) => {
  console.log('Worker exited with code:', code);
  if (code !== 0) {
    process.exit(code);
  }
});

// Send the conversion request
const request = {
  scadFilePath: testScadPath,
  libraryFiles: {},
  exportFormat: 'svg',
  timeout: 30000
};

console.log('Sending request:', JSON.stringify(request, null, 2));
worker.send(request);
