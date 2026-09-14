# AI Skills

Robot Developer Extensions for URDF includes AI skills that help GitHub Copilot work with URDF, Xacro, and OpenSCAD. Each skill is a focused set of instructions for a particular task, such as choosing robot geometry, creating reusable macros, or checking a rendered part after an edit.

Skills provide reusable workflows rather than a separate AI model. Copilot can load relevant skills as needed, and you can ask for a skill by name in your prompt.

## Getting Started

1. Install **Robot Developer Extensions for URDF** and enable GitHub Copilot Chat in a supported version of VS Code. The extension currently requires VS Code 1.110 or later.
2. Open your project and the `.urdf`, `.xacro`, or `.scad` file you want to work on.
3. Run **URDF: Preview** from the Command Palette to make the model available for visual feedback.
4. Open Copilot Chat in **Agent** mode and describe your task. Include the target file, dimensions, units, and any constraints.
5. Enable the URDF MCP tools in Chat when you want Copilot to inspect previews, validate OpenSCAD, or discover libraries. Review any tool approval prompts before proceeding.

For example:

> Use the openscad-development skill to add four mounting holes to this bracket. Use millimeters, keep the existing outer dimensions, and check the result with a preview screenshot.

The registered skills are contributed by the extension; you do not need to copy them into each project. Skill selection depends on your request and the capabilities of your Chat client. Naming a skill explicitly helps make your intent clear.

## Included Skills

These seven skills are registered with VS Code through the extension's `chatSkills` contribution:

| Skill | When to use it | Example request |
| --- | --- | --- |
| `urdf-fundamentals` | Create or debug robot descriptions, links, joints, coordinate frames, inertial properties, and package references. | “Check this robot's joint hierarchy, limits, and coordinate frames.” |
| `urdf-geometry` | Choose between basic shapes, custom OpenSCAD geometry, and existing meshes, with simpler collision geometry where possible. | “Simplify this robot's collision geometry without changing its visual appearance.” |
| `xacro-conversion` | Replace repeated URDF structures with reusable Xacro macros and shared parameters. The workflow asks for approval before conversion. | “Convert these four wheel definitions into a reusable Xacro macro.” |
| `openscad-development` | Create or modify `.scad` files using an edit, screenshot, compare, and refine loop. | “Make this housing taller and verify the rendered result.” |
| `openscad-customizer` | Add Customizer parameter controls, part selection, and patterned `MxN` parts for batch export. | “Add grouped size controls and individually exportable parts to this model.” |
| `openscad-integration` | Add non-trivial parametric geometry to URDF/Xacro, including mesh references and millimeter-to-meter scaling. | “Create an OpenSCAD sensor bracket and integrate its STL into my robot.” |
| `openscad-librarian` | Discover available libraries, recommend existing modules, and guide installation and configuration when a library is missing. | “What installed library can help me create a rounded enclosure?” |

### OpenSCAD Customizer Skill

The `openscad-customizer` skill covers parameter controls, groups, sliders, dropdowns, part selection, and patterned `MxN` parts for batch export.

Example request:

> Use the openscad-customizer skill to add grouped size controls and a part selector to this model, keeping an assembly preview and individually exportable parts.

See the [OpenSCAD guide](OpenSCAD.md) for Customizer and export functionality.

## Companion Agents

The extension contributes two specialized agents through `chatAgents`. Select one from Chat's agent picker when available:

| Agent | Focus |
| --- | --- |
| `urdf-agent` | Robot descriptions, kinematic structure, geometry selection, and visual verification across URDF, Xacro, and OpenSCAD. |
| `openscad-artist` | Parametric OpenSCAD design, library reuse, and iterative visual refinement. |

An **agent** sets the overall role and available tools for a conversation. A **skill** supplies a task-specific workflow. **MCP tools** give the assistant access to operations such as rendering and validation. These work together: an agent can use relevant skills and check its work through the URDF MCP server.

## Visual Feedback and Validation

The OpenSCAD development workflow is designed to check the result, not just generate code:

1. Create or modify the model.
2. Use the URDF MCP screenshot tools to inspect the rendered geometry.
3. Compare the result with the requested shape, placement, proportions, and appearance.
4. Fix discrepancies and repeat.
5. Use `validate_openscad` to check for OpenSCAD compilation errors before declaring the work complete.

The MCP server also provides `get_openscad_libraries` so the assistant can inspect available library documentation before inventing new geometry. Third-party libraries are not automatically installed just because a skill mentions them; configure additional locations with `urdf-editor.OpenSCADLibraryPaths`.

The extension's embedded OpenSCAD renderer supports this workflow without a standalone OpenSCAD installation. ROS is not required for editing and previewing URDF/Xacro. See [Model Context Protocol](mcp.md) for preview-based AI examples and [Configuration](Configuration.md) for settings.

Screenshots and successful compilation are useful checks, but they do not prove mechanical fit, printability, or simulation correctness. Review generated code, dimensions, joint limits, and exported models before using them.

## Tips and Troubleshooting

- **Be specific:** Include filenames, dimensions, units, intended motion, and parts that must remain unchanged.
- **Start simple:** Ask for basic URDF shapes and simple collision geometry unless the task needs detailed meshes.
- **Skill not being used?** Mention its exact name and ask Copilot to follow it. Confirm the extension and Copilot Chat are enabled and that you have the latest extension version installed.
- **Preview tools unavailable?** Open **URDF: Preview**, check that the URDF MCP tools are enabled in Chat, and inspect the extension's output for errors.
- **Library missing?** Ask the librarian to inventory available libraries, then review the proposed installation or library-path changes.
- **Conversion proposed?** Review file renames and downstream references before approving a URDF-to-Xacro conversion.

## For Contributors

Skill instructions live in `assets/skills/<skill-name>/SKILL.md`; agent definitions live in `assets/agents/`. The extension declares discoverable skills in `package.json` under `contributes.chatSkills` and custom agents under `contributes.chatAgents`. Agent contribution paths point to the packaged files in `dist/assets/agents/`. Use `contributes.chatPromptFiles` for reusable prompts, not agent definitions. When adding or changing a skill or agent, update its registration and this guide together so the documented inventory matches what users can discover.