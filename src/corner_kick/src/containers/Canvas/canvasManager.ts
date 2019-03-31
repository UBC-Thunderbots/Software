import * as PIXI from 'pixi.js';

import Viewport from 'pixi-viewport';

import { SPRITESHEET } from 'SRC/constants/canvas';
import { ILayer } from 'SRC/types';

import { Layer } from './layer';
import { LayerReceiver } from './layerReceiver';
import { SpritePool } from './spritePool';
import { SpritesheetManager } from './spritesheetManager';

export class CanvasManager {
    private app: PIXI.Application;
    private viewport: Viewport;

    private spritePool: SpritePool;
    private spritesheetManager: SpritesheetManager;

    private layerReceiver: LayerReceiver;
    private layerDict: { [key: number]: Layer } = {};
    private newLayerCallback: (id: number) => void;

    constructor(newLayerCallback: (id: number) => void) {
        this.initView();

        this.spritePool = new SpritePool();
        this.spritesheetManager = new SpritesheetManager(SPRITESHEET);

        this.layerReceiver = new LayerReceiver(this.handleLayerMessage);
        this.layerReceiver.connect();

        this.newLayerCallback = newLayerCallback;
    }

    public getView = () => {
        return this.app.view;
    };

    public resize = (width: number, height: number) => {
        this.app.renderer.resize(width, height);
        this.viewport.resize(width, height);

        this.app.render();
    };

    public handleLayerOperations = (layers: ILayer[]) => {
        this.viewport.removeChildren();

        layers.forEach((layer) => {
            if (layer.visible) {
                this.viewport.addChild(this.layerDict[layer.id].getLayerObject());
            }
        });
    };

    private initView = () => {
        this.app = new PIXI.Application({
            width: 100,
            height: 100,
            antialias: true,
            transparent: true,
        });

        this.viewport = new Viewport({
            screenWidth: 100,
            screenHeight: 100,
            worldWidth: 100,
            worldHeight: 100,

            interaction: this.app.renderer.plugins.interaction,
        });
        this.viewport.drag().wheel();

        this.app.stage.addChild(this.viewport);
    };

    private handleLayerMessage = (data: ArrayBuffer) => {
        const incomingDataView = new DataView(data);

        const layer = incomingDataView.getUint8(0);
        if (this.layerDict[layer] === undefined) {
            this.layerDict[layer] = new Layer(
                layer,
                this.spritePool,
                this.spritesheetManager,
            );

            this.viewport.addChild(this.layerDict[layer].getLayerObject());
            this.newLayerCallback(layer);
        }

        this.layerDict[layer].handleLayerMessage(data);
    };
}
