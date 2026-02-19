import { createOpenSCAD } from 'openscad-wasm-prebuilt';

(async () => {
  try {
    console.log('Creating OpenSCAD instance...');
    const openscad = await createOpenSCAD({
      print: (text) => console.log('OpenSCAD:', text),
      printErr: (text) => console.error('OpenSCAD Error:', text),
    });

    const instance = openscad.getInstance();
    
    // Test SVG export
    console.log('Testing SVG export...');
    instance.FS.writeFile('/test.scad', 'square([10, 10]);');
    
    try {
      console.log('\n=== Calling with --help to see available options ===');
      instance.callMain(['--help']);
    } catch (e) {
      // Help typically exits with non-zero
    }
    
    console.log('\n=== Attempting SVG export ===');
    try {
      instance.callMain(['-o', '/output.svg', '/test.scad']);
      const svg = instance.FS.readFile('/output.svg', { encoding: 'utf8' });
      console.log('✓ Success! SVG output length:', svg.length);
      console.log('First 500 chars of SVG:\n', svg.substring(0, 500));
    } catch (e) {
      console.error('✗ SVG export failed:', e.message || e);
    }
    
  } catch (error) {
    console.error('Error:', error);
  }
})();
