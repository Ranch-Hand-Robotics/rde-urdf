// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as vscode from 'vscode';
import * as path from "path";

import URDFPreview from './preview';

export default class URDFPreviewManager implements vscode.WebviewPanelSerializer {
    private readonly _previews: URDFPreview[] = [];
    private _activePreview: URDFPreview | undefined = undefined;
    private _context: vscode.ExtensionContext;
    private _trace: vscode.OutputChannel;

    public constructor(context: vscode.ExtensionContext, trace: vscode.OutputChannel) {
        this._context = context;
        this._trace = trace;
    }

    public setContext(context: vscode.ExtensionContext)
    {
        this._context = context;
    }

    public refresh() {
        for (const preview of this._previews) {
            preview.refresh();
        }
    }

    public preview(
        resource: vscode.Uri
    ): void {
        if (URDFPreviewManager.handlesUri(resource)) {
            let preview = this.getExistingPreview(resource);
            if (preview) {
                preview.reveal();
            } else {
                preview = this.createNewPreview(this._context, resource);
            }

            preview.update(resource);
        }
    }

    public get activePreviewResource() {
        return this._activePreview && this._activePreview.resource;
    }

    public async deserializeWebviewPanel(
        webview: vscode.WebviewPanel,
        state: any
    ): Promise<void> {
        if (state) {
            const preview = await URDFPreview.revive(
                webview,
                this._context,
                this._trace,
                state);

            this.registerPreview(preview);
        }
    }

    private getExistingPreview(
        resource: vscode.Uri
    ): URDFPreview | undefined {
        return this._previews.find(preview =>
            preview.matchesResource(resource));
    }

    private createNewPreview(
        context: vscode.ExtensionContext,
        resource: vscode.Uri
    ): URDFPreview {
        const preview = URDFPreview.create(
            context,
            resource,
            this._trace);

        this._activePreview = preview;
        return this.registerPreview(preview);
    }

    private registerPreview(
        preview: URDFPreview
    ): URDFPreview {
        this._previews.push(preview);

        preview.onDispose(() => {
            const existing = this._previews.indexOf(preview);
            if (existing === -1) {
                return;
            }

            this._previews.splice(existing, 1);
            if (this._activePreview === preview) {
                this._activePreview = undefined;
            }
        });
        
        preview.onDidChangeViewState(({ webviewPanel }) => {
            this._activePreview = webviewPanel.active ? preview : undefined;
        });

        return preview;
    }

    private static handlesUri(
        uri: vscode.Uri
    ) : boolean {

        let ext = path.extname(uri.fsPath);
        if (ext === ".xacro" ||
            ext === ".urdf") {
                return true;
            }

        return false;
    }
}
