# License Compatibility and Compliance Guide

## Overview

This VS Code extension has a complex licensing situation due to the inclusion of both MIT and GPL-2.0 licensed components. This document explains the implications and requirements.

## Main Extension License

The core extension code is licensed under the **MIT License** (see [LICENSE](LICENSE) file).

## GPL-2.0 Dependencies

This extension includes the following GPL-2.0 licensed dependency:

- **openscad-wasm-prebuilt** (version 1.2.0)
  - Source: https://github.com/lorenzowritescode/openscad-wasm
  - License: GPL-2.0-or-later
  - Used for: Converting OpenSCAD (.scad) files to STL for 3D preview

## Legal Implications

### For End Users
- You can freely use this extension
- No additional obligations when using the extension in VS Code

### For Distributors and Redistributors
When redistributing this extension (including bundling in other software), you must comply with **both** licenses:

1. **MIT License requirements**: Include copyright notice and license text
2. **GPL-2.0 requirements**: 
   - Include source code or provide written offer for source code
   - Ensure all distributed components remain under GPL-compatible licenses
   - Include this notice and the full GPL-2.0 license text

### For Developers Modifying the Extension

If you modify this extension:

1. **Non-OpenSCAD components**: Can be modified and distributed under MIT terms
2. **OpenSCAD-related components**: Must be distributed under GPL-2.0 terms
3. **Combined distribution**: The entire combined work must be distributed under GPL-2.0 terms due to GPL's "viral" nature

## Source Code Availability

As required by GPL-2.0, the complete source code for this extension is available at:
https://github.com/Ranch-Hand-Robotics/rde-urdf

The source code for the openscad-wasm-prebuilt dependency is available at:
https://github.com/lorenzowritescode/openscad-wasm

## Compliance Checklist

When distributing this extension, ensure you:

- [ ] Include the MIT license text for the main extension
- [ ] Include the GPL-2.0 license text (found in THIRD_PARTY_NOTICES.txt)
- [ ] Provide source code access or written offer for source code
- [ ] Include attribution for openscad-wasm-prebuilt
- [ ] Ensure any modifications to OpenSCAD-related functionality remain GPL-2.0 compatible

## Alternative Licensing

If the GPL-2.0 requirements are incompatible with your use case, consider:

1. **Removing OpenSCAD support**: The extension can be modified to remove OpenSCAD functionality, eliminating the GPL dependency
2. **Separate plugin architecture**: OpenSCAD functionality could be moved to a separate, optional plugin
3. **Different OpenSCAD integration**: Use a different OpenSCAD integration method that doesn't require GPL libraries

## Questions?

For licensing questions, consult with a qualified legal professional. This document is for informational purposes only and does not constitute legal advice.