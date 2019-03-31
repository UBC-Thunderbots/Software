/**
 * This file specifies a CanvasManager class. It manages the layers
 * in the Canvas, handles new messages from the websocket and handle resize events.
 */

import * as PIXI from 'pixi.js';

import Viewport from 'pixi-viewport';

import { ILayer, ISpritesheet } from 'SRC/types';
import { Pool } from 'SRC/utils/pool';

import { Layer } from './layer';
import { SpritesheetManager } from './spritesheetManager';

/**
 * The CanvasManager handles most Canvas operations. Namely, it manages
 * the layers being displayed in the Canvas as well as resize events.
 */
export class CanvasManager {
    private app: PIXI.Application;
    private viewport: Viewport;

    private spritePool: Pool<PIXI.Sprite>;
    private spritesheetManager: SpritesheetManager;

    private layerDict: { [key: number]: Layer } = {};

    /**
     * Creates a new CanvasManager and initializes the sprite pool and the
     * spritesheet.
     * @param spritesheet The spritesheet to load in memory
     */
    constructor(spritesheet: ISpritesheet) {
        this.initView();

        this.spritePool = new Pool(() => {
            const sprite = new PIXI.Sprite();
            sprite.anchor.set(0.5, 0.5);
            return sprite;
        });
        this.spritesheetManager = new SpritesheetManager(spritesheet);
    }

    /**
     * Gets the HTML Canvas element. Used by the Canvas React component.
     */
    public getView = () => {
        return this.app.view;
    };

    /**
     * Resizes the view to fit the new dimensions. Call when the browser window
     * is resized.
     * @param width the new width of the container
     * @param height the new height of the container
     */
    public resize = (width: number, height: number) => {
        this.app.renderer.resize(width, height);
        this.viewport.resize(width, height);

        // We call render to avoid the screen flickering during the
        // resize operation
        this.app.render();
    };

    /**
     * Updates the layer order and visibility
     * @param layers the new layer information
     */
    public handleLayerOperations = (layers: ILayer[]) => {
        this.viewport.removeChildren();

        // Only add layers that are visible
        layers.forEach((layer) => {
            if (layer.visible) {
                this.viewport.addChild(this.layerDict[layer.id].getLayerObject());
            }
        });
    };

    /**
     * Parse and update a layer with new shape data. Creates the layer
     * if necessary.
     * @param data the layer data to parse
     * @param newLayerCallback callback called when a new layer is detected
     */
    public handleLayerMessage = (
        data: ArrayBuffer,
        newLayerCallback: (id: number) => void,
    ) => {
        const incomingDataView = new DataView(data);

        // Get layer id
        const id = incomingDataView.getUint8(0);

        // If we have not seen this layer id before, we create a new layer object
        if (this.layerDict[id] === undefined) {
            // Add the layer to the view
            this.layerDict[id] = new Layer(id, this.spritePool, this.spritesheetManager);
            this.viewport.addChild(this.layerDict[id].getLayerObject());

            // Notify user that we received a new layer
            newLayerCallback(id);
        }

        // Update the layer with new shape data
        this.layerDict[id].handleLayerMessage(data);
    };

    /**
     * Initializes the Canvas view
     */
    private initView = () => {
        this.app = new PIXI.Application({
            // We use 100px width and height
            // as a default until we know the dimensions of the container
            width: 100,
            height: 100,
            antialias: true,
            transparent: true,
        });

        this.viewport = new Viewport({
            // We use 100px width and height
            // as a default until we know the dimensions of the container
            screenWidth: 100,
            screenHeight: 100,
            worldWidth: 1000,
            worldHeight: 1000,

            interaction: this.app.renderer.plugins.interaction,
        });
        this.viewport.drag().wheel();

        this.app.stage.addChild(this.viewport);
    };
}
