---
name: openscad-customizer
description: Use this skill when modifying OpenSCAD files where the user wants customizer features, Parts export, patterned MxN parts, parameter parsing, UI generation, or conversion to STL/SVG/GLB with parameter overrides.
---

# OpenSCAD Customization
This skill provides utilities for working with OpenSCAD customizer features, including:
- Parsing OpenSCAD files for customizer variables
- Generating UI controls based on variable types and metadata
- Converting OpenSCAD files to STL/SVG/GLB with parameter overrides

## Parts
The RDE-URDF extension includes special support for "Parts" arrays, allowing users to define multiple variants of a model in a single SCAD file. The customizer will parse these arrays and generate UI controls for selecting between different part configurations, and allow exporting of the individual parts as separate STL/SVG/GLB files. This enables a powerful workflow for users to manage complex models with multiple configurations in a single source file.

```scad
// Part Selection
part = "assembly"; // [assembly, corner, front, back, side]
```

*Assembly* mode would render the full model, while selecting *corner* would only render the corner piece, and so on. The customizer UI will automatically generate a dropdown control for the `part` variable, allowing users to easily switch between configurations and export the desired variant.

### Patterned `MxN` Parts

A part option containing the literal, case-sensitive token `MxN` is an export pattern rather than a single part. For example:

```scad
// Part Selection
part = "assembly"; // [assembly, bottom_MxN]
```

When exporting `bottom_MxN`, the exporter substitutes zero-based integer coordinates and probes parts with **N as the inner iteration**:

1. `bottom_0x0`
2. `bottom_0x1`
3. `bottom_0x2` — if this generates no geometry, the current N row ends
4. `bottom_1x0`
5. `bottom_1x1`
6. Continue N until that row generates no geometry
7. Continue M until an `Mx0` probe generates no geometry

The no-geometry probes are expected boundaries and are not exported or reported as failed parts. Exported files use the expanded part names, such as `bottom_0x0.stl` and `bottom_1x1.stl`.

Model each row as a contiguous sequence beginning at N=0. Do not leave gaps: if `bottom_0x1` is empty, the exporter will not probe `bottom_0x2`. Likewise, every M row must begin at N=0, and an empty `bottom_2x0` ends the entire `bottom_MxN` pattern.

The SCAD model must generate geometry for each valid expanded name and intentionally generate no top-level geometry for out-of-range names:

```scad
// Part Selection
part = "assembly"; // [assembly, bottom_MxN]

if (part == "assembly") {
	assembly();
} else if (part == "bottom_0x0") {
	bottom_tile(0, 0);
} else if (part == "bottom_0x1") {
	bottom_tile(0, 1);
} else if (part == "bottom_1x0") {
	bottom_tile(1, 0);
} else if (part == "bottom_1x1") {
	bottom_tile(1, 1);
}
// All other bottom_MxN expansions intentionally produce no geometry.
```

When creating or reviewing patterned parts:

- Use exactly one literal `MxN` token in the part option.
- Start both dimensions at zero.
- Keep valid N values contiguous within each M row.
- Ensure out-of-range selections produce no geometry rather than placeholder geometry.
- Ensure iteration eventually terminates; exporters enforce a safety limit against patterns that always generate output.

## Defining Customizable Parameters

Parameters are parsed from top-level variable assignments in your `.scad` file. Follow the standard OpenSCAD Customizer conventions:

**Labels** — place a single-line comment immediately before the assignment to add a description:
```scad
// pressure angle
P = 45;
```

**Groups** — use a block comment of the form `/* [Group Name] */` to organize parameters into named sections:
```scad
/* [Slider] */
// slider widget for number
slider = 34; // [10:100]
```

**Sliders** — append an inline range comment `// [min:max]` to create a slider. Use `// [min:step:max]` for a stepped slider:
```scad
slider = 34;     // [10:100]
stepSlider = 2;  // [0:5:100]
```

**Dropdowns** — provide a comma-separated list of values to create a combo box. Works for both numbers and strings:
```scad
Numbers = 2;      // [0, 1, 2, 3]
Strings = "foo";  // [foo, bar, baz]
```

**Checkboxes** — boolean variables automatically render as a checkbox:
```scad
Variable = true;
```

**Textboxes** — string variables become a text input. Append `// [N]` to constrain the input length:
```scad
String = "length"; // [8]
```

**Vectors** — vector variables are supported and can include a range to constrain each component:
```scad
Vector3 = [12, 34, 46]; // [0:2:50]
```

**Hiding parameters** — variables declared inside a `/* [Hidden] */` section are excluded from the Customizer panel:
```scad
/* [Hidden] */
debugMode = true;
```

## Issues with programatically defined Variables
The customizer relies on static analysis of the SCAD file, so variables defined programmatically (e.g. inside a function or generated via a loop) will not be detected. As a workaround, you can define "dummy" variables with the same names and default values at the top level of your file, along with comments to ensure they are included in the customizer:

```scad
// Define dummy variables for programmatically generated parameters
// width = 10
// height = 20
// radius = 5
cube([width, height, 1]);
cylinder(r=radius, h=height);
```
Additionally, variables whose values are assigned via expressions that cannot be evaluated at parse time (e.g. `size = baseSize * 2;`) will not have their values reflected in the customizer UI will generate a warning. In these cases, the customizer will still create controls for the variable, but it will not be able to determine the initial value or enforce any constraints based on the expression. Users will need to manually adjust the values in the UI, and you may want to provide clear documentation in your SCAD file to explain how these variables are intended to be used.

To prevent the warning use the following workaround:
```scad
variable = 10; // Default value for customizer
// variable = baseSize * 2; // Original expression that cannot be evaluated at parse time
module hidden() { } // never called

variable = baseSize * 2; // Re-assign with original expression inside a module to avoid parse-time evaluation

```