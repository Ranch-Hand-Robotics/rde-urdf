import * as vscode from 'vscode';
import * as path from 'path';
import * as fs from 'fs';

function getDocumentUriFromCommandArg(uri?: vscode.Uri): vscode.Uri | undefined {
  if (uri) {
    return uri;
  }
  return vscode.window.activeTextEditor?.document.uri;
}

function extractOpenSCADParts(scadText: string): string[] {
  const normalizeValue = (raw: string): string => raw
    .replace(/\\"/g, '"')
    .replace(/\\'/g, "'")
    .replace(/\\n/g, '\n')
    .replace(/\\t/g, '\t')
    .trim();

  const extractQuotedStrings = (source: string): string[] => {
    const stringRegex = /"((?:\\.|[^"\\])*)"|'((?:\\.|[^'\\])*)'/g;
    const values: string[] = [];
    let match: RegExpExecArray | null;

    while ((match = stringRegex.exec(source)) !== null) {
      const raw = match[1] ?? match[2] ?? '';
      const normalized = normalizeValue(raw);
      if (normalized.length > 0) {
        values.push(normalized);
      }
    }

    return values;
  };

  const arrayAssignmentMatch = scadText.match(/(^|\n)\s*(?:part|parts)\s*=\s*\[([\s\S]*?)\]\s*;/i);
  if (arrayAssignmentMatch) {
    return extractQuotedStrings(arrayAssignmentMatch[2]);
  }

  const dropdownConstraintMatch = scadText.match(/(^|\n)\s*part\s*=\s*(?:"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|[^;]+?)\s*;\s*\/\/\s*\[([^\]]+)\]/i);
  if (!dropdownConstraintMatch) {
    return [];
  }

  return dropdownConstraintMatch[2]
    .split(',')
    .map(value => normalizeValue(value))
    .filter(value => value.length > 0);
}

function isLikely2DPart(partName: string): boolean {
  const name = partName.toLowerCase();

  if (/(\b3d\b|\bstl\b|solid|volume)/i.test(name)) {
    return false;
  }

  return /(\b2d\b|\bsvg\b|laser\s*cut|lasercut|flat|outline|profile|sheet|panel|plate)/i.test(name);
}

export function getPreferredExportFormats(partName: string): Array<'stl' | 'svg'> {
  return isLikely2DPart(partName) ? ['svg', 'stl'] : ['stl', 'svg'];
}

function sanitizePartNameForFilename(partName: string): string {
  const sanitized = partName
    .replace(/[<>:"/\\|?*\x00-\x1F]/g, '_')
    .replace(/\s+/g, '_')
    .replace(/_+/g, '_')
    .replace(/^_+|_+$/g, '');

  return sanitized.length > 0 ? sanitized : 'part';
}

/**
 * Register OpenSCAD export commands (STL, SVG, and Parts)
 */
export function registerOpenSCADExportCommands(
  context: vscode.ExtensionContext,
  tracing: vscode.OutputChannel
): void {
  const exportOpenSCADStlCommand = vscode.commands.registerCommand("urdf-editor.exportScadStl", async (uri?: vscode.Uri) => {
    const documentUri = getDocumentUriFromCommandArg(uri);
    if (!documentUri) {
      return;
    }

    if (path.extname(documentUri.fsPath).toLowerCase() !== '.scad') {
      vscode.window.showErrorMessage('This command is only available for .scad (OpenSCAD) files');
      return;
    }

    try {
      const { exportOpenSCAD } = await import('./openscad');

      await vscode.window.withProgress({
        location: vscode.ProgressLocation.Notification,
        title: 'Exporting OpenSCAD to STL',
        cancellable: true
      }, async (progress, token) => {
        progress.report({ message: 'Converting .scad to .stl...' });
        
        const stlPath = await exportOpenSCAD(documentUri.fsPath, 'stl', tracing, token);
        
        if (stlPath) {
          vscode.window.showInformationMessage(`STL exported successfully: ${stlPath}`);
        } else if (!token.isCancellationRequested) {
          vscode.window.showErrorMessage('Failed to export STL. Check the output panel for details.');
        }
      });
    } catch (error) {
      vscode.window.showErrorMessage(`Failed to export STL: ${error instanceof Error ? error.message : String(error)}`);
      tracing.appendLine(`STL export error: ${error instanceof Error ? error.stack : String(error)}`);
    }
  });
  context.subscriptions.push(exportOpenSCADStlCommand);

  const exportOpenSCADSvgCommand = vscode.commands.registerCommand("urdf-editor.exportSVG", async (uri?: vscode.Uri) => {
    const documentUri = getDocumentUriFromCommandArg(uri);
    if (!documentUri) {
      return;
    }

    if (path.extname(documentUri.fsPath).toLowerCase() !== '.scad') {
      vscode.window.showErrorMessage('SVG export is only available for .scad (OpenSCAD) files');
      return;
    }

    try {
      const { exportOpenSCAD } = await import('./openscad');
      
      await vscode.window.withProgress({
        location: vscode.ProgressLocation.Notification,
        title: 'Exporting OpenSCAD to SVG',
        cancellable: true
      }, async (progress, token) => {
        progress.report({ message: 'Converting .scad to .svg...' });
        
        const svgPath = await exportOpenSCAD(documentUri.fsPath, 'svg', tracing, token);
        
        if (svgPath) {
          vscode.window.showInformationMessage(`SVG exported successfully: ${svgPath}`);
          
          const openFile = await vscode.window.showInformationMessage(
            'Would you like to open the exported SVG file?',
            'Open',
            'Cancel'
          );
          
          if (openFile === 'Open') {
            const svgUri = vscode.Uri.file(svgPath);
            await vscode.commands.executeCommand('vscode.open', svgUri);
          }
        } else if (!token.isCancellationRequested) {
          vscode.window.showErrorMessage('Failed to export SVG. Check the output panel for details.');
        }
      });
    } catch (error) {
      vscode.window.showErrorMessage(`Failed to export SVG: ${error instanceof Error ? error.message : String(error)}`);
      tracing.appendLine(`SVG export error: ${error instanceof Error ? error.stack : String(error)}`);
    }
  });
  context.subscriptions.push(exportOpenSCADSvgCommand);

  const exportOpenSCADPartsCommand = vscode.commands.registerCommand("urdf-editor.exportScadParts", async (uri?: vscode.Uri) => {
    const documentUri = getDocumentUriFromCommandArg(uri);
    if (!documentUri) {
      return;
    }

    if (path.extname(documentUri.fsPath).toLowerCase() !== '.scad') {
      vscode.window.showErrorMessage('This command is only available for .scad (OpenSCAD) files');
      return;
    }

    try {
      const scadText = await fs.promises.readFile(documentUri.fsPath, 'utf8');
      const parts = extractOpenSCADParts(scadText);

      if (parts.length === 0) {
        vscode.window.showErrorMessage('No part list found. Add something like: part = "assembly"; // [assembly, t_edge, mid]');
        return;
      }

      const { exportOpenSCAD } = await import('./openscad');

      await vscode.window.withProgress({
        location: vscode.ProgressLocation.Notification,
        title: 'Exporting OpenSCAD parts',
        cancellable: true
      }, async (progress, token) => {
        const exportedPaths: string[] = [];
        const failedParts: string[] = [];

        for (let i = 0; i < parts.length; i++) {
          const part = parts[i];
          const preferredFormats = getPreferredExportFormats(part);

          progress.report({
            message: `Exporting part ${i + 1}/${parts.length}: ${part}`,
            increment: 100 / parts.length
          });

          let outputPath: string | null = null;
          let outputFormat: 'stl' | 'svg' | null = null;

          for (const format of preferredFormats) {
            outputPath = await exportOpenSCAD(
              documentUri.fsPath,
              format,
              tracing,
              token,
              {
                parameterOverrides: {
                  part
                },
                suppressErrorMessage: preferredFormats.length > 1
              }
            );

            if (token.isCancellationRequested) {
              return;
            }

            if (outputPath) {
              outputFormat = format;
              if (format !== preferredFormats[0]) {
                tracing.appendLine(`Part '${part}' exported as ${format.toUpperCase()} after fallback from ${preferredFormats[0].toUpperCase()}.`);
              }
              break;
            }
          }

          if (!outputPath || !outputFormat) {
            failedParts.push(part);
            continue;
          }

          const dir = path.dirname(outputPath);
          const baseName = path.basename(documentUri.fsPath, '.scad');
          const partName = sanitizePartNameForFilename(part);
          const desiredPath = path.join(dir, `${baseName}.${partName}.${outputFormat}`);

          try {
            await fs.promises.rm(desiredPath, { force: true });
            await fs.promises.rename(outputPath, desiredPath);
            exportedPaths.push(desiredPath);
          } catch (renameError) {
            tracing.appendLine(`Failed to rename exported part file '${outputPath}' -> '${desiredPath}': ${renameError instanceof Error ? renameError.message : String(renameError)}`);
            exportedPaths.push(outputPath);
          }
        }

        if (exportedPaths.length > 0) {
          const summary = failedParts.length > 0
            ? `Exported ${exportedPaths.length}/${parts.length} parts. Failed: ${failedParts.join(', ')}`
            : `Exported ${exportedPaths.length} part file(s) successfully.`;
          vscode.window.showInformationMessage(summary);
        } else if (!token.isCancellationRequested) {
          vscode.window.showErrorMessage('No parts were exported. Check the output panel for details.');
        }
      });
    } catch (error) {
      vscode.window.showErrorMessage(`Failed to export parts: ${error instanceof Error ? error.message : String(error)}`);
      tracing.appendLine(`Parts export error: ${error instanceof Error ? error.stack : String(error)}`);
    }
  });
  context.subscriptions.push(exportOpenSCADPartsCommand);
}
