# OpenSCAD Libraries Quick Reference

## Popular Libraries Overview

### MCAD (OpenSCAD Standard Library)
**Status:** Core library, widely available  
**Best for:** Mechanical parts, fasteners, common shapes

| Module | Purpose | Common Parameters |
|--------|---------|-------------------|
| `boxes::box()` | Parametric box with finger joints | size, wall, finger depth |
| `boxes::roundedBox()` | Box with rounded corners | size, round radius |
| `gears::involute_gear()` | Gear generation | teeth, module, pressure angle |
| `motors::stepper_nema()` | Stepper motor models | size (17, 23, 34) |
| `bearing::ball_bearing()` | Ball bearing models | diameter, width |

**Install:** Comes with most OpenSCAD installations  
**Import:** `use <MCAD/boxes.scad>;` or `use <MCAD/gears.scad>;`

### BOSL2 (Belfry OpenSCAD Library 2)
**Status:** Modern, actively maintained, comprehensive  
**Best for:** Advanced geometry, rounded shapes, mathematical functions

| Module | Purpose | Common Parameters |
|--------|---------|-------------------|
| `rounding::rounded_cube()` | Cube with rounded edges | size, r (radius) |
| `rounding::rounded_prism()` | Prism with rounded edges | vertices, height, r |
| `shapes::rounded_cylinder()` | Cylinder with rounded edges | h, r, rounding |
| `shapes::convex_hull()` | Convex hull of multiple shapes | children |
| `sweep::path_sweep()` | Sweep profile along path | shape, path |
| `masks::frustum()` | Tapered shapes | height, base, top, sides |
| `attachments::attach()` | Parametric positioning | anchor, orient, spin |

**Install:** [https://github.com/BelfrySCAD/BOSL2](https://github.com/BelfrySCAD/BOSL2)  
**Import:** `include <BOSL2/std.scad>;`

**Key strength:** Advanced transformation tools and parametric attachment system

### NopSCADlib
**Status:** Large, well-maintained, real-world parts  
**Best for:** Mechanical assemblies, realistic components

| Category | Examples |
|----------|----------|
| Power | Power supplies, USB connectors, batteries |
| Mechanical | Motors, gearboxes, pulleys, belts |
| Electronics | Switches, buttons, LED modules, sensors |
| Fasteners | Screws, nuts, bolts (all variants) |
| Structural | Extrusions, frames, brackets |

**Install:** [https://github.com/nopscadlib/nopscadlib](https://github.com/nopscadlib/nopscadlib)  
**Import pattern:** Highly modular with organized folder structure

**Key strength:** Huge library of real, parametric mechanical components

### Customizer Libraries
**Status:** Great for parametric design  
**Best for:** User-facing, configurable designs

- **Customizer-compatible** — Libraries that work with OpenSCAD Customizer interface
- **Well-parameterized** — All dimensions exposed as variables
- **Documentation** — Good examples and parameter explanations

## Library Comparison by Use Case

### I need rounded corners/edges
1. **Best:** BOSL2 `rounded_cube()`, `rounded_cylinder()` — Superior control
2. **Alternative:** MCAD `roundedBox()` — Simpler but less flexible
3. **Custom:** Write your own if specific corner profiles needed

### I need common mechanical parts
1. **Best:** NopSCADlib — Real-world accuracy, extensive selection
2. **Alternative:** MCAD bearing/motor modules for basics
3. **Fallback:** Search Thingiverse or GitHub for specific components

### I need mathematical shapes
1. **Best:** BOSL2 shapes and math modules
2. **Alternative:** MCAD for basic geometry
3. **Custom:** Parametric functions if specialized math required

### I need a complete assembly
1. **Best:** NopSCADlib — Pre-built assemblies with examples
2. **Alternative:** Combine multiple BOSL2 modules
3. **Composition:** Mix libraries strategically

## Installation Quick Start

### Windows
```
1. Create folder: %USERPROFILE%\Documents\OpenSCAD\libraries
2. Clone/download library into that folder
3. Restart OpenSCAD or reload files
```

### macOS
```
1. Create folder: ~/Documents/OpenSCAD/libraries
2. Clone/download library into that folder
3. Restart OpenSCAD or reload files
```

### Linux
```
1. Create folder: ~/.local/share/OpenSCAD/libraries
2. Clone/download library into that folder
3. Restart OpenSCAD or reload files
```

### VS Code Configuration
Add custom library paths in VS Code settings:
```json
"urdf-editor.OpenSCADLibraryPaths": [
  "${workspaceFolder}/my_libraries",
  "~/.local/share/OpenSCAD/libraries"
]
```

## Code Examples

### BOSL2 Rounded Cube
```scad
include <BOSL2/std.scad>;

rounded_cube([50, 40, 30], r=5);
```

### MCAD Gear
```scad
use <MCAD/gears.scad>;

// 20 tooth gear, 2mm module, pressure angle 20°
involute_gear(20, 2, 20, 10);
```

### NopSCADlib Motor Mount
```scad
include <NopSCADlib/lib.scad>;

// Nema 17 stepper motor
nema(17);

// Mounting bracket
motor_plate(17, 5);  // size, thickness
```

## Tips for Library Usage

1. **Read the documentation** — Library authors usually provide examples
2. **Understand parameters** — Know what each parameter controls
3. **Test rendering** — Verify output matches expectations
4. **Use variables** — Don't hardcode values, make geometry parametric
5. **Combine libraries** — Mix and match modules from different libraries
6. **Version awareness** — Some libraries have breaking changes between versions
7. **Performance** — Complex libraries can be slow to render; optimize iteratively

## Resources

| Resource | Purpose |
|----------|---------|
| [OpenSCAD Language Reference](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual) | Official language docs |
| [Thingiverse OpenSCAD](https://www.thingiverse.com/search?q=filetype:scad) | Community shared designs |
| [OpenSCAD GitHub Topic](https://github.com/topics/openscad) | Find new libraries and projects |
| Library GitHub pages | Specific library documentation |
| Customizer interface in OpenSCAD | Test parametric designs interactively |

## Troubleshooting

### Library not found
- **Check path:** Verify library is in correct folder for your OS
- **Restart:** Restart OpenSCAD or reload VS Code extension
- **Check import:** Verify `use` or `include` statement syntax
- **Workspace settings:** Check `urdf-editor.OpenSCADLibraryPaths` in VS Code

### Module name conflicts
- **Namespace with comments:** Use path in comment to disambiguate
- **Rename locally:** Create wrapper that renames conflicting modules
- **Library order:** Control which library loads first if possible

### Unexpected rendering
- **Missing parameters:** Check if module requires specific parameter format
- **Type mismatches:** Ensure numbers vs arrays (e.g., `size=50` vs `size=[50,50,50]`)
- **Scale issues:** Verify dimensions are in expected units (usually mm)

## Workflow Decision Tree

```
Need geometry?
├─ Check get_openscad_libraries output
├─ Search for matching module
├─ Found?
│  ├─ YES → Use it, show example code
│  └─ NO → Check alternative libraries
├─ Still not found?
│  ├─ Can compose from library modules? → Show composition example
│  └─ Recommend custom code with library building blocks
```
