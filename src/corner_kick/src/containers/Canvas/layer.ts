/**
 * This file specifies a Layer class. This class is responsible
 * for handling sprite creation and layer message parsing.
 */

import * as PIXI from 'pixi.js';

import { BYTES_PER_SHAPE } from 'SRC/constants';

import { LayerParsingException } from 'SRC/utils/exceptions';
import { Pool } from 'SRC/utils/pool';

import { SpritesheetManager } from './spritesheetManager';

/**
 * Manages the layer, parses layer messages and handle sprite creation/
 * destruction
 */
export class Layer {
    private id: number;
    private spritePool: Pool<PIXI.Sprite>;
    private spritesheetManager: SpritesheetManager;

    private layerObject: PIXI.Container;

    /**
     * Creates a new Layer object. Requires a shared sprite pool and spritesheet
     * to create new sprites when parsing layer data.
     * @param id The id of the layer
     * @param spritePool a sprite pool for sprite allocation
     * @param spritesheetManager a spritesheet object with preloaded textures
     */
    constructor(
        id: number,
        spritePool: Pool<PIXI.Sprite>,
        spritesheetManager: SpritesheetManager,
    ) {
        this.id = id;
        this.spritePool = spritePool;
        this.spritesheetManager = spritesheetManager;

        this.layerObject = new PIXI.Container();
    }

    /**
     * Get the underlying Canvas object for rendering
     */
    public getLayerObject = () => {
        return this.layerObject;
    };

    /**
     * Parses and updates the sprites contained in this layer. Clears any
     * past layer data received by this layer.
     * @param data the layer data to parse
     */
    public handleLayerMessage = (data: ArrayBuffer) => {
        const incomingDataView = new DataView(data);

        // Parse the layer number
        const layer = incomingDataView.getUint8(0);
        const incomingSpriteCount = (data.byteLength - 1) / BYTES_PER_SHAPE;

        // Make sure the layer number is correct
        if (layer !== this.id) {
            throw new LayerParsingException(
                'I received the wrong layer id, ' +
                    'check that you are calling the right layer',
            );
            // Make sure we received a correct number of bytes
        } else if (incomingSpriteCount !== Math.round(incomingSpriteCount)) {
            throw new LayerParsingException(
                'I received an invalid number of bytes, ' +
                    'the message is probably malformed',
            );
        }

        // Add/remove allocated sprites in layer if needed
        const currSpriteCount = this.layerObject.children.length;
        if (incomingSpriteCount === 0) {
            // Special case, simply unallocate all sprites
            const oldSprites = this.layerObject.children as PIXI.Sprite[];
            this.spritePool.unallocate(oldSprites);
            this.layerObject.removeChildren();
        } else if (incomingSpriteCount > currSpriteCount) {
            // We allocated what we are missing
            const newSprites = this.spritePool.allocate(
                incomingSpriteCount - currSpriteCount,
            );
            this.layerObject.addChild(...newSprites);
        } else if (incomingSpriteCount < currSpriteCount) {
            // We clear all sprites from the parent
            const oldSprites = this.layerObject.children as PIXI.Sprite[];
            this.spritePool.unallocate(oldSprites);
            this.layerObject.removeChildren();

            // And add back what we need
            const newSprites = this.spritePool.allocate(incomingSpriteCount);
            this.layerObject.addChild(...newSprites);
        }

        const sprites = this.layerObject.children as PIXI.Sprite[];
        sprites.forEach((sprite, index) => {
            // We start at 1 as the first byte of the message
            // contain the layer number
            const startPos = 1 + index * BYTES_PER_SHAPE;

            // Get all the fields for the shape...
            const textureId = incomingDataView.getUint8(startPos);
            const x = incomingDataView.getInt16(startPos + 1);
            const y = incomingDataView.getInt16(startPos + 3);
            const width = incomingDataView.getInt16(startPos + 5);
            const height = incomingDataView.getInt16(startPos + 7);
            const rotation = incomingDataView.getInt16(startPos + 9);
            const opacity = incomingDataView.getUint8(startPos + 11);
            const red = incomingDataView.getUint8(startPos + 12);
            const green = incomingDataView.getUint8(startPos + 13);
            const blue = incomingDataView.getUint8(startPos + 14);

            // ...And set them for the sprite
            // Fetch the texture from the spritesheet based on the ID we received
            const texture = this.spritesheetManager.getTextureByID(textureId);
            if (texture === null) {
                throw new LayerParsingException(
                    'I received an texture id that isn not loaded in the spritesheet',
                );
            } else {
                sprite.texture = texture;
            }

            // Because the anchor is the in middle of the sprite, we need to
            // calculate the sprite x and y position to simulate a top-left corner
            // positioning system
            sprite.x = x + width / 2;
            sprite.y = y + height / 2;

            sprite.width = width;
            sprite.height = height;

            // We receive rotation in tenth of degrees
            // Rotation is radians so we need to perform the degree to rad conversion
            sprite.rotation = (rotation / 10 / 180) * Math.PI;

            // Alpha is from 0 to 1 so we need to remap the value we received
            sprite.alpha = opacity / 255;

            // Tint accepts a number of format 0xRRGGBB
            sprite.tint = ((red & 0xff) << 16) | ((green & 0xff) << 8) | (blue & 0xff);
        });
    };
}
