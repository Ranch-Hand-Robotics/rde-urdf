# Library Installation & Configuration Guide

## Automated Installation via MCP (if available)

The URDF Editor extension includes MCP server support for library management:

### Prerequisites
- VS Code with URDF Editor extension installed
- MCP server running (auto-starts when needed)
- Internet connection for downloading libraries

### Workflow
1. Use `get_openscad_libraries` to discover available libraries
2. If library not available, request installation
3. MCP server handles:
   - Downloading from source (GitHub, official repositories)
   - Extracting to correct OS-specific folder
   - Updating VS Code configuration if needed
   - Verifying installation success

### Configuration
Set custom library paths in VS Code `.vscode/settings.json`:
```json
"urdf-editor.OpenSCADLibraryPaths": [
  "${workspaceFolder}/libraries",
  "${workspaceFolder}/third-party-libs",
  "~/Documents/OpenSCAD/libraries"
]
```

**Variables supported:**
- `${workspaceFolder}` — Root of current workspace
- `~` or `${HOME}` — User home directory
- Absolute paths — Windows: `C:\\Libraries`, Unix: `/opt/libraries`

## Manual Installation

### Step 1: Identify Library Location
Get the source of the library you want:
- GitHub repository URL
- Official library source
- Community repository link

Common sources:
- [MCAD on GitHub](https://github.com/openscad/MCAD)
- [BOSL2 on GitHub](https://github.com/BelfrySCAD/BOSL2)
- [NopSCADlib on GitHub](https://github.com/nopscadlib/nopscadlib)

### Step 2: Download/Clone

#### Option A: Git Clone (Recommended)
```bash
# Navigate to library folder (create if needed)
cd ~/Documents/OpenSCAD/libraries  # macOS/Linux
# or
cd %USERPROFILE%\Documents\OpenSCAD\libraries  # Windows

# Clone the library
git clone https://github.com/BelfrySCAD/BOSL2.git
```

**Advantage:** Easy updates with `git pull`

#### Option B: Manual Download
1. Visit library GitHub page
2. Click "Code" → "Download ZIP"
3. Extract to library folder
4. Rename folder if needed to match expected name

**Note:** No easy updates with this method

### Step 3: Verify Installation

#### In OpenSCAD
```scad
use <BOSL2/std.scad>;

// If this renders without error, library is accessible
cube([10, 10, 10]);
```

#### In VS Code
Create a test file and open the Preview:
1. Create `test-library.scad` in workspace
2. Add `use <LibraryName/module.scad>;` at top
3. Run **URDF: Preview** command
4. If geometry appears without errors, library is working

### Step 4: Add to VS Code Settings (Optional)

If installing to a custom location, add to `.vscode/settings.json`:

```json
{
  "urdf-editor.OpenSCADLibraryPaths": [
    "${workspaceFolder}/my-libraries",
    "~/Documents/OpenSCAD/libraries"
  ]
}
```

## Library Folder Structure

### Standard Installation Structure
```
~/Documents/OpenSCAD/libraries/
├── BOSL2/
│   ├── BOSL2/
│   │   ├── rounding.scad
│   │   ├── shapes.scad
│   │   └── ...
│   ├── README.md
│   └── LICENSE
├── MCAD/
│   ├── boxes.scad
│   ├── gears.scad
│   └── ...
└── NopSCADlib/
    ├── modules/
    ├── examples/
    └── lib.scad
```

### Workspace-Local Libraries
```
my-project/
├── libraries/          # Optional: local libraries
│   ├── my-parts/
│   │   └── brackets.scad
│   └── utilities/
│       └── helpers.scad
├── src/
│   └── assembly.scad   # uses <my-parts/brackets.scad>
└── .vscode/
    └── settings.json   # configures workspace library path
```

## Common Installation Issues

### Issue: "Library not found" error

**Cause 1: Incorrect folder structure**
- **Solution:** Verify folder name matches import statement
  - File: `~/Documents/OpenSCAD/libraries/BOSL2/std.scad`
  - Import: `use <BOSL2/std.scad>;` ✓

**Cause 2: Library folder path not configured**
- **Solution:** Add to VS Code settings:
```json
"urdf-editor.OpenSCADLibraryPaths": ["~/Documents/OpenSCAD/libraries"]
```

**Cause 3: Library not in standard location**
- **Solution:** Check your OS:
  - Windows: `%USERPROFILE%\Documents\OpenSCAD\libraries`
  - macOS: `~/Documents/OpenSCAD/libraries`
  - Linux: `~/.local/share/OpenSCAD/libraries`

### Issue: Module not found within library

**Cause 1: Incorrect module path**
- **Check:** Library documentation for correct module path
- **Example:** BOSL2 modules are in `BOSL2/` subdirectory
  - Correct: `use <BOSL2/rounding.scad>;`
  - Incorrect: `use <rounding.scad>;`

**Cause 2: Missing dependency**
- **Solution:** Some libraries depend on others
- **Check:** Library README for dependencies
- **Action:** Install dependencies first

**Cause 3: Library version mismatch**
- **Solution:** Ensure compatible library versions
- **Update:** `git pull` in library folder
- **Check:** Library CHANGELOG for breaking changes

### Issue: Rendering is very slow

**Cause:** Complex libraries with many modules loaded

**Solutions:**
1. Use `use` instead of `include` (only loads needed parts)
2. Use specific module imports, not entire library
3. Reduce geometry complexity in test
4. Break design into smaller test files

### Issue: Conflicting module names

**When:** Two libraries have modules with same name

**Solutions:**
1. Rename locally:
```scad
use <BOSL2/rounding.scad> as bosl2_rounding;
```

2. Wrap in comment to disambiguate:
```scad
use <BOSL2/rounding.scad>;  // BOSL2/rounding
use <MCAD/rounding.scad>;   // MCAD/rounding
```

3. Load only what you need:
```scad
use <BOSL2/rounding.scad>;
// Skip conflicting MCAD module
```

## Updating Libraries

### Via Git (Recommended)
```bash
cd ~/Documents/OpenSCAD/libraries/BOSL2
git pull origin main
```

**Benefit:** See what changed, easy rollback

### Manual Update
1. Download new version from GitHub
2. Backup old folder: `mv BOSL2 BOSL2.backup`
3. Extract new version
4. Test rendering
5. Delete backup if working

## Repository-Specific Setup

### BOSL2 (GitHub)
```bash
git clone https://github.com/BelfrySCAD/BOSL2.git
# Include entire BOSL2 folder with:
include <BOSL2/std.scad>;
```

### MCAD (Usually comes pre-installed)
```bash
# If not installed:
git clone https://github.com/openscad/MCAD.git
# Use with:
use <MCAD/boxes.scad>;
```

### NopSCADlib (GitHub)
```bash
git clone https://github.com/nopscadlib/nopscadlib.git
# Include with:
include <nopscadlib/lib.scad>;
```

## Troubleshooting Checklist

- [ ] Library folder is in a standard location or configured in VS Code
- [ ] Folder name exactly matches the path in `use`/`include` statement
- [ ] Library dependencies are installed
- [ ] No conflicting module names (or properly disambiguated)
- [ ] Restarted OpenSCAD or reloaded VS Code after installation
- [ ] File permissions allow reading library files
- [ ] No special characters or spaces in folder path (use underscore instead)
- [ ] Verified with test `.scad` file that library loads

## Performance Optimization

### For large libraries with many modules
```scad
// Good: Load only what you need
use <BOSL2/rounding.scad>;
use <BOSL2/shapes.scad>;

// Slower: Load entire library with dependencies
include <BOSL2/std.scad>;
```

### For workspace with many libraries
Consider organizing into categories:
```
libraries/
├── mechanical/      # Load only when needed
├── utilities/       # Core utilities, always load
├── experimental/    # Rarely used
```

## Next Steps

1. **Inventory available libraries** with `get_openscad_libraries`
2. **Choose primary library** for your project type
3. **Verify installation** with test rendering
4. **Add to VS Code settings** if using custom path
5. **Start creating** with library modules
