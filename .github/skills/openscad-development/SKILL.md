---
name: openscad-development
description: OpenSCAD development workflow with screenshot validation for openscad
---

# OpenSCAD Development with Screenshot Validation

This skill guides the iterative development of OpenSCAD geometry with built-in validation through screenshot comparison.

## Core Workflow: Validate Every Change

**This is the mandatory workflow for ALL OpenSCAD code changes:**

1. **Make the code change** - Create or modify the .scad file
2. **Take a screenshot** - Use MCP server to capture the rendered geometry
3. **Compare with requirements** - Does the screenshot match what was asked?
4. **Iterate if needed** - Attempt fixes or ask user for clarification

### The Validation Loop

```
User Request
    ↓
Create/Modify .scad file
    ↓
Take Screenshot (mcp_urdf_take_screenshot)
    ↓
Does it match user requirements? ──→ YES → Done ✓
    ↓ NO
Analyze discrepancies
    ↓
Can I fix it automatically? ──→ YES → Modify .scad and loop back
    ↓ NO
Ask user for clarification
```

## Important: MCP Server Rendering

**Do NOT assume OpenSCAD is installed locally.** The MCP Server has an embedded OpenSCAD renderer that provides all needed functionality:

- ✅ Use `mcp_urdf_take_screenshot()` - Renders the current file
- ✅ Use `mcp_urdf_take_screenshot_by_filename()` - Renders a specific file  
- ✅ The MCP Server handles all rendering without needing the standalone OpenSCAD application

This means you can validate OpenSCAD changes without requiring the user to install anything locally. All geometry validation happens through the MCP Server's embedded renderer.

## Key Rules

### ✅ ALWAYS DO THIS

1. **After every code change** - Take a screenshot to validate the result
2. **Analyze screenshots** - Compare against what the user requested
3. **Attempt auto-fixes first** - If something's wrong, try to fix it yourself
4. **Document issues** - When asking for clarification, explain what's wrong
5. **Loop until correct** - Keep iterating until the output matches requirements

### ❌ NEVER DO THIS

1. ❌ Assume the code will render correctly without verification
2. ❌ Skip screenshot validation steps
3. ❌ Ask the user to verify when you can check yourself
4. ❌ Create code and walk away without validation
5. ❌ Ignore discrepancies between expected and actual output
6. ❌ Assume the standalone OpenSCAD application is installed
7. ❌ Tell the user to "check this in OpenSCAD" instead of using MCP Server screenshots

## Screenshots: What to Look For

When comparing a screenshot with user requirements, evaluate:

### Geometry Correctness
- [ ] Shape is correct (box, cylinder, complex form, etc.)
- [ ] Dimensions look reasonable
- [ ] Proportions match requested spec
- [ ] Symmetry is correct (if applicable)

### Positioning & Orientation
- [ ] Part is positioned where it should be
- [ ] Rotation/angle is correct
- [ ] Alignment with other components is right
- [ ] Center/origin placement is appropriate

### Technical Aspects
- [ ] Resolution ($fn) is adequate (smooth curves vs faceted)
- [ ] No rendering artifacts or holes
- [ ] Boolean operations (union/difference) worked correctly
- [ ] Color/material displays correctly (if applicable)

### Assembly/Integration
- [ ] Interfaces with adjacent parts properly
- [ ] Mounting points line up
- [ ] Clearances are appropriate
- [ ] Overall scale looks correct

## Using Screenshot Tools

**The MCP Server provides the ONLY rendering pipeline needed. Use these tools exclusively for validation:**

### Single File Screenshot

```
Use: mcp_urdf_take_screenshot_by_filename()
When: You want to preview a specific SCAD file
Returns: Screenshot of that file's rendered geometry
Rendering: Handled by MCP Server's embedded OpenSCAD renderer
```

**Example workflow:**
```
1. Create/edit shell.scad
2. Take screenshot to verify it rendered
3. If wrong, modify and take another screenshot
4. Repeat until correct
```

### Current Active File Screenshot

```
Use: mcp_urdf_take_screenshot()
When: You're actively editing and want quick validation
Returns: Screenshot of current workspace file being edited
Rendering: Handled by MCP Server's embedded OpenSCAD renderer
```

**No local OpenSCAD installation required.** All rendering happens through the MCP Server.

## Validation Strategies

### Strategy 1: Simple Geometry Changes

**User**: "Make the cylinder 20mm taller"

```
1. Modify the height parameter: height = 50 (was 30)
2. Take screenshot
3. Compare: Is it proportionally taller? Does it look right?
4. If YES → Done
5. If NO → Adjust parameter and retry
```

### Strategy 2: Complex Changes

**User**: "Add a cutout for a sensor mount in the center"

```
1. Create geometry for the cutout (dimensions from spec)
2. Add it to a difference() operation
3. Take screenshot
4. Check:
   - Position: Is it centered correctly? ✓/✗
   - Size: Does it look like the right size? ✓/✗
   - Integration: Does it cut cleanly? ✓/✗
5. If any ✗, adjust parameters:
   - Wrong position? → Adjust translate() values
   - Wrong size? → Adjust cube/cylinder dimensions
   - Integration problem? → Review boolean operation
6. Take another screenshot and verify
7. Repeat until all checks pass
```

### Strategy 3: Assembly Verification

**User**: "Position multiple parts with proper spacing"

```
1. Create part placement code
2. Take screenshot
3. Verify checklist:
   - Part A positioned correctly? ✓/✗
   - Part B positioned correctly? ✓/✗
   - Spacing between parts adequate? ✓/✗
   - Overall alignment correct? ✓/✗
4. For each ✗:
   - Adjust positioning parameters (translate/rotate)
   - Take new screenshot
   - Verify that specific issue
5. Iterate on each issue independently
```

## Common Issues & Fixes

### Issue: Part not visible
**Check:**
- Is it outside the view bounds? → Adjust translate/position
- Is it behind something? → Check z-order in code
- Is it scaled wrong? → Verify scale() operations
- Is it a rendering issue? → Try increasing $fn

**Fix:** Adjust positioning or visibility, take screenshot

### Issue: Dimensions look wrong
**Check:**
- Are proportions off? → Compare with spec dimensions
- Is scaling applied? → Check for scale() or unit mismatch
- Are parameters set correctly? → Verify variable values

**Fix:** Update parameter values, take screenshot

### Issue: Boolean operations not working
**Check:**
- Do shapes actually overlap? → Verify positions
- Is it difference/union/intersection intended? → Check logic
- Are solids closed? → Can't subtract from open geometry

**Fix:** Review operation, adjust geometry, take screenshot

### Issue: Detail too smooth or too faceted
**Check:**
- Is $fn set appropriately? → Increase for smooth, decrease for speed
- Should specific shapes have different resolution? → Use $fn override

**Fix:** Adjust $fn value, take screenshot

## Iterative Refinement Example

**User**: "Create a mounting bracket with a 3×3 grid of mounting holes spaced 20mm apart"

### Iteration 1: Basic layout
```scad
// mounting_bracket.scad
// 3×3 grid of M3 mounting holes, 20mm spacing

module mounting_hole() {
  // M3 hole: 3.2mm diameter
  circle(r=1.6);
}

module bracket() {
  // Mounting plate with 9 holes
  difference() {
    // Base plate
    cube([80, 80, 5]);
    
    // Hole grid: 3 rows × 3 columns
    for (row = [0:2]) {
      for (col = [0:2]) {
        translate([col * 20 + 10, row * 20 + 10, -1])
          linear_extrude(7)
            mounting_hole();
      }
    }
  }
}

bracket();
```

**Take screenshot** → Check layout visually
- [ ] 9 holes visible? 
- [ ] Grid alignment looks even?
- [ ] Proper spacing?
- [ ] Hole size appropriate for M3?

**Result**: Layout looks off - holes too close to edges

### Iteration 2: Adjust positioning
```scad
// Shift hole positions inward
for (row = [0:2]) {
  for (col = [0:2]) {
    translate([col * 20 + 12, row * 20 + 12, -1])  // Changed from 10 to 12
      linear_extrude(7)
        mounting_hole();
  }
}
```

**Take screenshot** → Better! Edges look balanced

### Iteration 3: Add mounting features
```scad
module bracket() {
  difference() {
    union() {
      // Base plate
      cube([80, 80, 5]);
      
      // Corner reinforcement
      for (x = [5, 75], y = [5, 75]) {
        translate([x, y, 0])
          cylinder(h=10, r=4);
      }
    }
    
    // Hole grid (same as before)
    for (row = [0:2]) {
      for (col = [0:2]) {
        translate([col * 20 + 12, row * 20 + 12, -1])
          linear_extrude(7)
            mounting_hole();
      }
    }
    
    // Center mounting point
    translate([40, 40, 0])
      cylinder(h=5, r=2.5);
  }
}
```

**Take screenshot** → Verify reinforcement looks good, mounting point is clear

### Iteration 4: Final polish
**Compare screenshot with requirements:**
- [ ] Dimensions match specification?
- [ ] Hole placement and spacing correct?
- [ ] Thickness appropriate for application?
- [ ] Overall design is functional?

**Result**: Ready for use!

## When to Ask for Clarification

**Ask the user when:**

1. **Ambiguous requirements** - "You said 'round corners' but how much radius? 5mm? 10mm?"
2. **Multiple valid interpretations** - Screenshot could be improved several ways; which do you prefer?
3. **Conflicting constraints** - "That position would interfere with the keyboard. Should I move A or B?"
4. **Beyond current scope** - The change requires new components or major rework outside the request
5. **Screenshot clearly wrong** - You've tried 3 approaches and none match the user's intent

**Always include in clarification request:**
- What the current screenshot shows
- What the requirements appeared to be
- Why it doesn't match
- 1-2 alternative approaches you considered

## Project-Specific Customization

This skill is designed to work with any OpenSCAD project. When working on specific projects:

1. **Understand the coordinate system** - Each project may have different conventions
2. **Know the component specifications** - Keep dimensions and constraints in mind
3. **Check for project libraries** - Reference shared parameter files and utility modules
4. **Verify integration** - Ensure new parts align and interface correctly with existing geometry
5. **Follow project conventions** - Maintain consistency with existing code style and organization

## Workflow Summary

### Quick Reference

```
For every OpenSCAD change:

1. Edit file (.scad)
2. Take screenshot (mcp_urdf_take_screenshot or mcp_urdf_take_screenshot_by_filename)
3. Compare with requirements
4. If wrong:
   - Analyze what's incorrect
   - Modify code to fix
   - Go to step 2
5. If unclear to user:
   - Document current state
   - Explain discrepancy
   - Ask specific clarification question
6. If correct:
   - Document the solution
   - Move to next task
```

### Tools Used

- `mcp_urdf_take_screenshot()` - Screenshot current active file
- `mcp_urdf_take_screenshot_by_filename(parameters)` - Screenshot specific file
- File editing tools - Modify .scad files
- `read_file()` - Reference existing code and specs

## Best Practices

### DO:
✅ Take a screenshot immediately after any code change
✅ Compare screenshots systematically against requirements
✅ Make small, targeted changes and verify each one
✅ Document what the screenshot shows vs. what was requested
✅ Attempt multiple approaches before asking for help
✅ Reference project specs and constraints in verification
✅ Keep iterating until the screenshot matches requirements

### DON'T:
❌ Make large changes and hope they work
❌ Skip screenshot validation
❌ Assume positioning is correct without visual verification
❌ Ask user to check something you can screenshot yourself
❌ Give up after one attempt when the output is wrong
❌ Ignore component integration issues in screenshots

## Summary

This skill enforces a tight feedback loop for OpenSCAD development:
- **Every code change gets a screenshot**
- **Every screenshot gets analyzed**
- **Issues get fixed automatically when possible**
- **Only ask user when necessary**

This approach ensures OpenSCAD geometry is developed correctly with visual validation at every step, enabling confident, iterative design workflows.
