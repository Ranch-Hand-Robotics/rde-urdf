import { createOpenSCAD } from 'openscad-wasm-prebuilt';
import * as fs from 'fs';

(async () => {
  try {
    console.log('Creating OpenSCAD instance...');
    const openscad = await createOpenSCAD({
      print: (text) => console.log('OpenSCAD:', text),
      printErr: (text) => console.error('OpenSCAD Error:', text),
    });

    const instance = openscad.getInstance();
    
    // Read test file
    const scadContent = await fs.promises.readFile('/tmp/test_2d_shape.scad', 'utf8');
    console.log('Read SCAD file:', scadContent.substring(0, 100));
    
    // Test SVG export
    console.log('\n=== Testing SVG export ===');
    instance.FS.writeFile('/input.scad', scadContent);
    
    try {
      instance.callMain(['-o', '/output.svg', '/input.scad']);
      const svg = instance.FS.readFile('/output.svg', { encoding: 'utf8' });
      console.log('✓ Success! SVG output length:', svg.length);
      console.log('SVG content:\n', svg);
      
      // Write to file
      await fs.promises.writeFile('/tmp/test_output.svg', svg, 'utf8');
      console.log('\n✓ SVG written to /tmp/test_output.svg');
    } catch (e) {
      console.error('✗ SVG export failed:', e.message || e);
    }
    
  } catch (error) {
    console.error('Error:', error);
  }
})();
