// Test script to demonstrate OpenSCAD documentation generation
import * as path from 'path';
import { generateAndSaveLibrariesDocumentation } from '../openscad';

async function testDocumentationGeneration() {
    const testDataPath = path.join(__dirname, 'testdata');
    const outputPath = path.join(testDataPath, 'test-openscad-docs.md');
    
    try {
        console.log('Generating OpenSCAD documentation...');
        await generateAndSaveLibrariesDocumentation(outputPath, testDataPath);
        console.log(`Documentation generated: ${outputPath}`);
    } catch (error) {
        console.error('Failed to generate documentation:', error);
    }
}

// Run test if this file is executed directly
if (require.main === module) {
    testDocumentationGeneration();
}