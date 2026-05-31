/**
 * OpenSCAD integration surface for rde-urdf.
 *
 * Provides a typed wrapper around babylon_ros OpenSCAD utilities
 * for file-based conversion in VS Code extensions.
 */

import * as impl from '@ranchhandrobotics/babylon_ros/openscad';

export type OpenSCADCustomizerValue = string | number | boolean | number[];

export interface OpenSCADCustomizerRangeConstraint {
  min?: number;
  max?: number;
  step?: number;
}

export interface OpenSCADCustomizerOption {
  value: string | number;
  label?: string;
}

export type OpenSCADCustomizerWidgetType =
  | 'slider'
  | 'spinbox'
  | 'checkbox'
  | 'dropdown'
  | 'vector'
  | 'textbox';

export interface OpenSCADCustomizerParseWarning {
  line: number;
  message: string;
}

export interface OpenSCADCustomizerVariable {
  name: string;
  valueType: 'string' | 'number' | 'boolean' | 'vector';
  defaultValue: OpenSCADCustomizerValue;
  tab?: string;
  hidden?: boolean;
  description?: string;
  widget?: OpenSCADCustomizerWidgetType;
  options?: OpenSCADCustomizerOption[];
  range?: OpenSCADCustomizerRangeConstraint;
  maxLength?: number;
  rawConstraint?: string;
  line?: number;
}

export interface OpenSCADCustomizerParseResult {
  variables: OpenSCADCustomizerVariable[];
  warnings: OpenSCADCustomizerParseWarning[];
  firstBraceLine?: number;
}

export interface OpenSCADParameterConfiguration {
  jsonContent: string;
  parameterSetName: string;
}

export interface OpenSCADValidationResult {
  valid: boolean;
  errors: string[];
  warnings: string[];
}

export interface OpenSCADLibraryModule {
  name: string;
  signature?: string;
  comment?: string;
  parameters?: string[];
}

export interface OpenSCADLibraryFile {
  filePath?: string;
  relativePath: string;
  headerComment?: string;
  modules: OpenSCADLibraryModule[];
}

export interface OpenSCADLibrariesDocumentation {
  generatedAt: string;
  libraryPaths: string[];
  libraries: OpenSCADLibraryFile[];
}

export const isOpenSCADFile = impl.isOpenSCADFile as (filePath: string) => boolean;
export const getDefaultOpenSCADLibraryPaths = impl.getDefaultOpenSCADLibraryPaths as () => string[];
export const getAllOpenSCADLibraryPaths = impl.getAllOpenSCADLibraryPaths as (
  workspaceRoot?: string,
  scadFilePathOrConfiguredPaths?: string[] | string,
) => Promise<string[]>;
export const convertOpenSCAD = impl.convertOpenSCAD as (
  scadFilePath: string,
  trace: any,
) => Promise<string | null>;
export const convertOpenSCADCancellable = impl.convertOpenSCADCancellable as (
  scadFilePath: string,
  trace: any,
  token?: any,
  options?: {
    timeout?: number;
    parameterOverrides?: Record<string, OpenSCADCustomizerValue>;
    parameterConfiguration?: OpenSCADParameterConfiguration;
    outputFormat?: 'stl' | 'glb';
  },
) => Promise<string | null>;
export const convertOpenSCADWithNodeWorker = impl.convertOpenSCADWithNodeWorker as (
  scadFilePath: string,
  trace?: any,
  options?: {
    timeout?: number;
    outputFormat?: 'stl' | 'glb';
    workspaceRoot?: string;
    configuredLibraryPaths?: string[];
    parameterOverrides?: Record<string, OpenSCADCustomizerValue>;
    parameterConfiguration?: OpenSCADParameterConfiguration;
  },
) => Promise<string | null>;
export const exportOpenSCAD = impl.exportOpenSCAD as (
  scadFilePath: string,
  exportFormat: 'stl' | 'svg',
  trace: any,
  token?: any,
  options?: {
    timeout?: number;
    parameterOverrides?: Record<string, OpenSCADCustomizerValue>;
    suppressErrorMessage?: boolean;
  },
) => Promise<string | null>;
export const generateOpenSCADLibrariesDocumentation = impl.generateOpenSCADLibrariesDocumentation as (
  workspaceRoot?: string,
) => Promise<OpenSCADLibrariesDocumentation>;
export const convertLibrariesDocumentationToMarkdown = impl.convertLibrariesDocumentationToMarkdown as (
  doc: OpenSCADLibrariesDocumentation,
) => string;
export const generateAndSaveLibrariesDocumentation = impl.generateAndSaveLibrariesDocumentation as (
  outputPath: string,
  workspaceRoot?: string,
) => Promise<void>;
export const validateOpenSCAD = impl.validateOpenSCAD as (
  scadFilePath: string,
  scadContent?: string,
  workspaceRoot?: string,
) => Promise<OpenSCADValidationResult>;
export const validateOpenSCADWithWorker = impl.validateOpenSCADWithWorker as (
  scadFilePath: string,
  scadContent: string,
  workspaceRoot?: string,
  trace?: any,
) => Promise<OpenSCADValidationResult>;
export const parseOpenSCADCustomizerVariables = impl.parseOpenSCADCustomizerVariables as (
  scadText: string,
) => OpenSCADCustomizerParseResult;
