# License Compatibility and Compliance Guide

## Overview

This VS Code extension includes components under different licenses. Most of the extension code is MIT-licensed, while OpenSCAD runtime components used for `.scad` conversion are GPL-2.0-or-later licensed. This document summarizes practical redistribution obligations for this mixed-license distribution.

## Main Extension License

The core extension code is licensed under the **MIT License** (see [LICENSE](LICENSE) file).

## GPL-2.0 Dependencies

This extension includes OpenSCAD runtime components under GPL-2.0-or-later:

- **openscad-wasm** (distributed through `@ranchhandrobotics/babylon_ros`)
  - Source: https://github.com/Ranch-Hand-Robotics/openscad-wasm
  - License: GPL-2.0-or-later
  - Used for: Converting OpenSCAD (`.scad`) files for preview/export features

Related integration source:
- `@ranchhandrobotics/babylon_ros`: https://github.com/Ranch-Hand-Robotics/babylon_ros

## Legal Implications

### For End Users
- You can freely use this extension
- No additional obligations when using the extension in VS Code

### For Distributors and Redistributors
When redistributing this extension (for example: repackaging a `.vsix`, mirroring binaries, or bundling with another product), you must comply with **both** license families:

1. **MIT License requirements**: Include copyright notice and license text
2. **GPL-2.0 requirements**: 
  - Provide complete corresponding source code (or a valid written offer where applicable)
  - Preserve GPL notices and disclaimers
  - Include GPL-2.0 license text and OpenSCAD attribution

### Distribution Model Clarification

- **Source-only modifications not distributed**: no redistribution obligation is triggered.
- **Distribution of extension artifacts that include OpenSCAD runtime collateral**: GPL obligations apply to that distributed combined package.
- **Distribution of a variant with OpenSCAD runtime removed**: MIT terms still apply to remaining code; GPL OpenSCAD runtime obligations do not apply to what is not distributed.

### For Developers Modifying the Extension

If you modify this extension:

1. **Non-OpenSCAD components**: Can be modified and distributed under MIT terms
2. **OpenSCAD runtime/integration components**: GPL-2.0-or-later obligations apply when distributed
3. **Combined distributed package**: Evaluate and comply with GPL requirements for the shipped combination

## Source Code Availability

The source code for this extension is available at:
https://github.com/Ranch-Hand-Robotics/rde-urdf

The source code for OpenSCAD runtime and integration components is available at:
- https://github.com/Ranch-Hand-Robotics/openscad-wasm
- https://github.com/Ranch-Hand-Robotics/babylon_ros

## Compliance Checklist

When distributing this extension, ensure you:

- [ ] Include the MIT license text for the main extension
- [ ] Include the GPL-2.0 license text (found in THIRD_PARTY_NOTICES.txt)
- [ ] Provide access to corresponding source code for distributed GPL-covered OpenSCAD runtime/integration components
- [ ] Include attribution for OpenSCAD WASM components and integration repositories
- [ ] Preserve license notices and disclaimers in redistributed artifacts

## Alternative Licensing

If the GPL-2.0 requirements are incompatible with your use case, consider:

1. **Removing OpenSCAD support**: The extension can be modified to remove OpenSCAD functionality, eliminating the GPL dependency
2. **Separate plugin architecture**: OpenSCAD functionality could be moved to a separate, optional plugin
3. **Different OpenSCAD integration**: Use a different OpenSCAD integration method that doesn't require GPL libraries

## Questions?

For licensing questions, consult with a qualified legal professional. This document is for informational purposes only and does not constitute legal advice.