---
name: openscad-customizer
description: Use this skill when modifying OpenSCAD files where the user wants to use the customizer features (parameter parsing, UI generation, and conversion to STL/SVG/GLB with parameter overrides).
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