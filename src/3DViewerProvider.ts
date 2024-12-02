import * as vscode from 'vscode';
import * as util from './utils';
import * as path from 'path';
import { Viewer3DDocument } from './3DViewerDocument';

export class Viewer3DProvider implements vscode.CustomReadonlyEditorProvider  {

	public static register(context: vscode.ExtensionContext): vscode.Disposable {
		const provider = new Viewer3DProvider(context);
		const providerRegistration = vscode.window.registerCustomEditorProvider(Viewer3DProvider.viewType, provider);
		return providerRegistration;
	}

	private static readonly viewType = 'urdf-editor.Viewer3D';

    private readonly _context: vscode.ExtensionContext;

    constructor(
            private readonly context: vscode.ExtensionContext
        ) { 
        this._context = context;
    }

	public openCustomDocument(uri: vscode.Uri, openContext: vscode.CustomDocumentOpenContext, token: vscode.CancellationToken): vscode.CustomDocument | Thenable<vscode.CustomDocument> {
		let doc = new Viewer3DDocument(this._context, uri);
		return doc;
	}


	public resolveCustomEditor(document: vscode.CustomDocument, webviewPanel: vscode.WebviewPanel, token: vscode.CancellationToken): void | Thenable<void> {
		const doc = document as Viewer3DDocument;
		doc.create(webviewPanel);
	}    
};
