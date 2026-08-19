import * as path from 'path';

export function resolveOpenSCADPartsOutputDirectory(
    sourceDirectory: string,
    workspaceRoot: string | undefined,
    configuredDirectory: string | undefined,
): string {
    const configured = configuredDirectory?.trim();
    if (!configured) {
        return sourceDirectory;
    }
    if (!workspaceRoot) {
        throw new Error('OpenSCAD parts output directory requires the SCAD file to be inside a workspace folder.');
    }
    if (path.isAbsolute(configured)) {
        throw new Error('OpenSCAD parts output directory must be relative to the workspace folder.');
    }

    const resolvedWorkspaceRoot = path.resolve(workspaceRoot);
    const expandedDirectory = configured.replace(/\$\{workspaceFolder\}/g, resolvedWorkspaceRoot);
    const resolvedOutputDirectory = path.resolve(resolvedWorkspaceRoot, expandedDirectory);
    const relativeToWorkspace = path.relative(resolvedWorkspaceRoot, resolvedOutputDirectory);
    if (relativeToWorkspace === '..' || relativeToWorkspace.startsWith(`..${path.sep}`) || path.isAbsolute(relativeToWorkspace)) {
        throw new Error('OpenSCAD parts output directory must stay inside the workspace folder.');
    }

    return resolvedOutputDirectory;
}
