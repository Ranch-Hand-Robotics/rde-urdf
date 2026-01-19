// Test script to demonstrate OpenSCAD validation
import * as path from 'path';
import { validateOpenSCAD } from '../openscad';

async function testValidation() {
    const testDataPath = path.join(__dirname, 'testdata');
    
    // Test valid file
    console.log('Testing valid OpenSCAD file...');
    const validFile = path.join(testDataPath, 'simple_cube.scad');
    const validResult = await validateOpenSCAD(validFile);
    console.log('Valid file result:', JSON.stringify(validResult, null, 2));
    
    // Test invalid file
    console.log('\nTesting invalid OpenSCAD file...');
    const invalidFile = path.join(testDataPath, 'invalid_syntax.scad');
    const invalidResult = await validateOpenSCAD(invalidFile);
    console.log('Invalid file result:', JSON.stringify(invalidResult, null, 2));
    
    // Test with content string
    console.log('\nTesting with content string...');
    const testContent = 'cube([10, 10, 10]);';
    const contentResult = await validateOpenSCAD('/tmp/test.scad', testContent);
    console.log('Content validation result:', JSON.stringify(contentResult, null, 2));
}

// Run test if this file is executed directly
if (require.main === module) {
    testValidation().catch(console.error);
}
