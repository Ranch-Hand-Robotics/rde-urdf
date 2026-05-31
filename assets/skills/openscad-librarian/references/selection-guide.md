# OpenSCAD Library Selection Guide

## Decision Framework

When a user needs geometry, guide them through this decision tree:

### Level 1: Identify the Need Category

```
What type of geometry is needed?

├─ Mechanical parts (gears, bearings, fasteners)
│  └─ → NopSCADlib, MCAD
│
├─ Rounded/smooth geometry (curved edges, hulls)
│  └─ → BOSL2 rounding, shapes
│
├─ Mathematical shapes (prisms, lofts, sweeps)
│  └─ → BOSL2 shapes, advanced geometry
│
├─ Common basic shapes (cubes, cylinders, spheres)
│  └─ → Built-in OpenSCAD (no library needed)
│
├─ Parametric assembly of components
│  └─ → NopSCADlib, or combine multiple libraries
│
├─ Artistic/organic geometry
│  └─ → BOSL2 transformations, custom algorithms
│
└─ Specialized domain (electrical, 3D printing, etc.)
   └─ → Search for domain-specific library
```

### Level 2: Find Matching Libraries

Once category is identified, search available libraries (via `get_openscad_libraries`) for:
- Module names matching the geometry type
- Parameter lists that support needed configurations
- Documentation showing usage examples

### Level 3: Evaluate Fit

For each candidate library, assess:

| Criterion | Questions | Weight |
|-----------|-----------|--------|
| **Completeness** | Does library have ALL needed modules? | High |
| **Parameterization** | Are dimensions/shapes fully configurable? | High |
| **Quality** | Is code well-maintained and tested? | High |
| **Simplicity** | Easy to use or steep learning curve? | Medium |
| **Performance** | Renders quickly enough? | Medium |
| **Documentation** | Examples available? | Medium |
| **Installation** | Already available or easy to install? | Low* |

*Installation barrier is low because MCP can automate it with user consent

## Common Geometry Requests & Recommendations

### Gear Design
**Best:** MCAD `gears::involute_gear()`
- **Why:** Industry standard, accurate tooth profiles, well-tested
- **Parameters:** teeth, module, pressure_angle, face_width
- **Usage:**
```scad
use <MCAD/gears.scad>;
involute_gear(20, 2, 20, 10);  // 20 teeth, 2mm module
```

**Alternative:** NopSCADlib gears (if higher fidelity assembly needed)

### Rounded Box/Cube
**Best:** BOSL2 `rounded_cube()`
- **Why:** Superior control, works well with transforms, modern design
- **Parameters:** size (array or scalar), r (corner radius), rounding (type)
- **Usage:**
```scad
include <BOSL2/std.scad>;
rounded_cube([50, 40, 30], r=5);
```

**Alternative:** MCAD `roundedBox()` (simpler but less flexible)

### Motor Mount
**Best:** NopSCADlib `motor_plate()`
- **Why:** Real mechanical components, pre-designed mounting patterns
- **Parameters:** size (motor NEMA size), thickness
- **Usage:**
```scad
include <nopscadlib/lib.scad>;
motor_plate(17, 5);  // NEMA 17, 5mm thick
```

### Bearing/Ball Joint
**Best:** NopSCADlib ball bearings or MCAD `bearing::ball_bearing()`
- **Why:** Accurate models matching real components
- **Parameters:** diameter, width, bore
- **Alternative:** BOSL2 if custom geometry needed

### Smooth Curved Surface/Sweep
**Best:** BOSL2 `path_sweep()` or `profile_sweep()`
- **Why:** Powerful transformation tools, smooth results
- **Parameters:** profile shape, path, transformations
- **Alternative:** Custom mesh if very specialized

### Parametric Bracket/Mount
**Best:** NopSCADlib or combine BOSL2 modules
- **Why:** Real mechanical design patterns, assembly-ready
- **Composition:** Combine rounding, positioning, hole generation

### Fastener (Screw, Bolt, Nut)
**Best:** NopSCADlib fasteners library
- **Why:** Real specifications, accurate models
- **Coverage:** M2 to M20, various head types, lock nuts
- **Alternative:** MCAD for basic needs

## Library Combinations That Work Well

### Mechanical Assembly
```scad
include <nopscadlib/lib.scad>;     // Parts library
use <BOSL2/rounding.scad>;          // Rounded features
use <BOSL2/masks.scad>;             // Cutouts, holes

// Design: Real parts + smooth geometry + precise features
```

### Artistic/Prototype
```scad
use <BOSL2/std.scad>;               // Core geometry
use <BOSL2/drawing.scad>;           // 2D drawing tools
use <BOSL2/transformations.scad>;   // Advanced transforms

// Design: Focus on smooth, parametric, creative forms
```

### Mixed Engineering
```scad
use <MCAD/gears.scad>;              // Mechanical gears
use <MCAD/boxes.scad>;              // Structural boxes
use <BOSL2/rounding.scad>;          // Polish edges

// Design: Standard parts + structured frame + aesthetic touches
```

## Evaluation Matrix

Use this matrix to compare libraries for a specific need:

```
Task: Create a rounded mounting bracket

Library Options:
1. BOSL2 (rounding + shapes)
2. NopSCADlib (real bracket designs)
3. Custom code

Evaluation:
                    BOSL2    NopSCADlib    Custom
Fits need?          80%      95%           100%
Easy to use?        70%      90%           50%
Time to implement?  Short    Very Short    Long
Customization?      High     Medium        Complete
Maintenance?        Easy     Easy          Hard
Learning curve?     Medium   Low           N/A

Winner: NopSCADlib (fastest, most appropriate)
```

## When to Create Custom Code Instead

Choose custom code if:

1. **Geometry doesn't exist in libraries**
   - No library module covers the need
   - Specialized, unique, domain-specific

2. **Library geometry needs heavy customization**
   - Extensive parameter tweaking required
   - Multiple transformations needed
   - Better to write custom for clarity

3. **Performance critical**
   - Library rendering too slow
   - Custom algorithm more efficient
   - Lightweight prototype better than feature-full

4. **Learning opportunity**
   - User wants to understand geometry creation
   - Educational value in custom code
   - Portfolio piece

5. **One-off design**
   - Single use, unlikely to reuse
   - Simple enough to custom code
   - Faster than learning library

## Red Flags (When NOT to Use a Library)

- Library is abandoned (no updates in 2+ years)
- Module has known bugs or issues
- Documentation is non-existent or wrong
- Installation is overly complicated
- Performance is unacceptable
- Library dependencies create conflicts

## Recommendation Checklist

Before recommending a library, verify:

- [ ] Library module exists for the need
- [ ] Module is actively maintained (recent commits)
- [ ] Documentation shows clear examples
- [ ] Parameters are documented and sensible
- [ ] Known issues don't block the use case
- [ ] Installation is feasible (in workspace or easy to add)
- [ ] Rendering performance is acceptable
- [ ] No conflicts with other libraries in project

## Example Recommendation Flow

**User:** "I need to create a custom joint mechanism with smooth curves"

**Librarian Steps:**
1. Query available libraries
2. Category: Smooth/advanced geometry → BOSL2
3. Check modules: `path_sweep()`, `profile_sweep()`, `rounded_prism()`
4. Evaluate fit: BOSL2 excellent for this use case
5. Install check: Is BOSL2 available? If not, offer installation
6. Provide example:
```scad
include <BOSL2/std.scad>;
profile_sweep(profile, path, uniform=true);
```
7. Explain parameters and customization options
8. Suggest parametric variations

## Maintenance & Updates

### Checking Library Health
```bash
# Check last commit
cd ~/Documents/OpenSCAD/libraries/BOSL2
git log -1 --format="%ai"  # Recent?

# Check issues
# Visit GitHub repository
# Look for open issues, unresolved PRs
```

### When to Update
- **Always:** Before using a library for critical design
- **Regularly:** Every 3-6 months for active projects
- **Test:** Verify rendering still works after update
- **Backup:** Keep backup of working version

### Version Pinning (for teams)
```
# If using git submodules, pin to specific commits
[submodule "libraries/BOSL2"]
    path = libraries/BOSL2
    url = https://github.com/BelfrySCAD/BOSL2.git
    branch = main
```

## Resource Links

### Official Library Repositories
- **BOSL2:** https://github.com/BelfrySCAD/BOSL2
- **MCAD:** https://github.com/openscad/MCAD
- **NopSCADlib:** https://github.com/nopscadlib/nopscadlib

### Discovery Resources
- **GitHub Topic:** https://github.com/topics/openscad
- **Thingiverse:** https://www.thingiverse.com/search?q=filetype:scad
- **OpenSCAD Website:** https://openscad.org/

### Documentation
- **BOSL2 Wiki:** https://github.com/BelfrySCAD/BOSL2/wiki
- **OpenSCAD Manual:** https://en.wikibooks.org/wiki/OpenSCAD_User_Manual
- **Library Documentation:** Check individual repository READMEs
