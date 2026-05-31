---
name: openscad-librarian
description: "Discover, recommend, and install OpenSCAD libraries for your projects. Use when: needing geometry functions, seeking existing modules before creating, managing library dependencies, or exploring what libraries are available in your workspace."
argument-hint: "What geometry or feature do you want to create? (optional: specify desired library category)"
user-invocable: true
disable-model-invocation: false
---

# OpenSCAD Library Librarian

Your expert guide for discovering, recommending, and installing OpenSCAD libraries. Instead of coding geometry from scratch, leverage the vast ecosystem of pre-built, tested, optimized library modules.

## Mission

Help you make informed decisions about which OpenSCAD libraries to use, understand what they offer, and install them into your workspace when needed—all with clear recommendations and user transparency.

## When to Use This Skill

- **"I need a rounded cube/gear/bearing mount/bracket"** → Find and suggest the right library
- **"What libraries do I have available?"** → Discover workspace library inventory
- **"Can I use an existing module for this geometry?"** → Library search before custom creation
- **"How do I add a new library to my workspace?"** → Installation and configuration
- **"Which library is best for X?"** → Compare alternatives and make recommendations
- **"How do I use this library?"** → Documentation and usage examples

## Procedure: Library Discovery & Recommendation

### Step 1: Inventory Available Libraries
Use the MCP tool `get_openscad_libraries` to retrieve comprehensive documentation of all libraries currently accessible in your workspace.

**What this gives you:**
- Complete list of available libraries
- Module/function signatures for each library
- Parameter descriptions and defaults
- Library categories and specializations
- Usage examples where documented

### Step 2: Analyze the Need
When a user requests geometry, ask these clarifying questions:
- What is the primary purpose? (mechanical, artistic, functional, prototype)
- What key features are needed? (rounded corners, parametric sizing, specific constraints)
- Are there performance considerations? (complexity, render time, file size)
- Any aesthetic preferences? (minimalist, detailed, organic)

### Step 3: Search & Recommend
Search the library inventory (from Step 1) for modules that match the requirements.

**Recommendation criteria:**
1. **Fit** — Does the library have what's needed?
2. **Quality** — Is it well-maintained and documented?
3. **Alternatives** — Are there competing options? When yes, compare them.
4. **Learning curve** — Is it easy to use or requires study?

**Present recommendations with:**
- Library name and primary purpose
- Specific modules that fit the need
- Code example showing usage
- Any limitations or setup requirements
- Installation status (already available vs. needs install)

### Step 4: Installation (If Needed)
If a recommended library isn't available, follow the installation workflow:

1. **Confirm with user** — "Library X isn't currently available. Would you like me to install it?"
2. **Document the source** — Identify library repository/location
3. **Use MCP for installation** — If MCP supports automated installation:
   - Prepare installation command
   - Execute with user's explicit consent (unless autopilot enabled)
   - Verify successful installation
4. **Fallback workflow** — If MCP doesn't support direct installation:
   - Provide manual installation instructions
   - Explain folder locations for their OS
   - Guide configuration in VS Code settings
5. **Verify accessibility** — After installation, confirm the library is accessible in the workspace

### Step 5: Provide Usage Guidance
Once library is available:
- Show complete code example of the recommended module
- Explain key parameters and what they control
- Highlight any common pitfalls or gotchas
- Suggest parametric variations they might try
- Link to library documentation if available

## Common Library Categories

### Mechanical Engineering (MCAD)
- **Gears** — Involute gears, spur gears, various tooth profiles
- **Bearings** — Ball bearings, roller bearings, bearing blocks
- **Fasteners** — Bolts, screws, nuts with accurate profiles
- **Boxes** — Parametric boxes with rounded corners, finger joints
- **Motors** — Common motor models (stepper, DC, servo)
- **Linear motion** — Rails, sliders, leadscrew profiles

### Advanced Geometry (BOSL2)
- **Rounding & filleting** — Smooth edges, bevels, rounded transforms
- **Sweep/loft** — Complex shapes by sweeping profiles
- **Convex hulls** — Organic shapes, collision geometry
- **Offset/shell** — Hollowing, thickness operations
- **Attachments** — Parametric positioning and alignment

### Specialized Libraries
- **MCAD/unibody** — Integrated component designs
- **NopSCADlib** — Large library of real-world parts and assemblies
- **Obrary** — Community-contributed modules
- **SCAD Plus** — Extended functionality and utilities

## Workflow Decision Tree

```
User requests geometry
    ├─ "Do I have a library that already does this?"
    │   ├─ YES → Recommend library module
    │   │   ├─ Already installed? → Show code example, proceed
    │   │   └─ Not installed? → Ask permission, install, show example
    │   └─ NO → Sketch what's needed, suggest creation approach
    │
    ├─ "What libraries are available to me?"
    │   └─ Run get_openscad_libraries, present organized list
    │
    └─ "How do I install a library?"
        └─ Provide OS-specific instructions and configuration
```

## Installation Paths (OS-Specific)

### Windows
- **Default library folder:** `%USERPROFILE%\Documents\OpenSCAD\libraries`
- **Alternative:** Configure via VS Code setting `urdf-editor.OpenSCADLibraryPaths`

### macOS
- **Default library folder:** `$HOME/Documents/OpenSCAD/libraries`

### Linux
- **Default library folder:** `$HOME/.local/share/OpenSCAD/libraries`

## MCP Server Integration

The URDF Editor extension includes MCP server tools for library management:

- **`get_openscad_libraries`** — Discover all available libraries and their contents
  - Returns comprehensive documentation
  - Includes module signatures and parameters
  - Shows usage patterns and examples
  - Identifies library categories and specializations

- **Library installation support** (if implemented) — Automated download and installation
  - Requires explicit user consent
  - Respects `urdf-editor.OpenSCADLibraryPaths` configuration
  - Handles OS-specific paths automatically

## Quality Assurance Checklist

When recommending a library, verify:
- [ ] Library is documented and maintained
- [ ] Module signatures are clear and well-parameterized
- [ ] Examples show common usage patterns
- [ ] No deprecated or broken modules in the recommendation
- [ ] Installation is straightforward (or provide clear guidance)
- [ ] User understands what the library does before installation

## Example Recommendation Session

**User:** "I need rounded cubes for my design."

**Librarian:**
1. Query available libraries → Find BOSL2, MCAD/boxes
2. Analyze needs → Rounded corners, parametric size, possibly different corner radii
3. Recommend → Show BOSL2's `rounded_cube()` module (superior to MCAD alternative)
4. Installation → Check if BOSL2 is available; if not, offer installation
5. Guidance → Provide code example with parameters explained
6. Variants → Show how to parametrize roundness, suggest size variations

```scad
use <BOSL2/rounding.scad>;

// Create a rounded cube: size=[width, depth, height], corner radius
rounded_cube([50, 40, 30], r=5);

// Parametric version
size = 50;
radius = 3;
rounded_cube([size, size, size], r=radius);
```

## When to Create Custom Code

If after library search no suitable module exists:
1. Document why the search failed (library missing, functionality gaps)
2. Propose custom creation approach
3. Suggest using library components as building blocks (compose/extend)
4. Explain advantages of custom code (control, optimization)
5. Consider suggesting the user request the library authors (open source feedback)

## Tips for Maximum Effectiveness

- **Know the library landscape** — Regularly review available libraries
- **Ask clarifying questions** — Understand the real need before recommending
- **Show code examples** — Make recommendations concrete and executable
- **Document decisions** — Help user understand why one library beats another
- **Verify installations work** — Test recommendations in the actual workspace
- **Link to sources** — Provide paths to library documentation and repositories
- **Suggest combinations** — Multiple libraries often work better together
