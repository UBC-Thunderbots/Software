import * as PIXI from 'pixi.js';

import { BYTES_PER_SHAPE } from 'SRC/constants';

import { LayerParsingException } from 'SRC/utils/exceptions';
import { SpritePool } from './spritePool';
import { SpritesheetManager } from './spritesheetManager';

export class Layer {
    private id: number;
    private spritePool: SpritePool;
    private spritesheetManager: SpritesheetManager;

    private layerObject: PIXI.Container;

    constructor(
        id: number,
        spritePool: SpritePool,
        spritesheetManager: SpritesheetManager,
    ) {
        this.id = id;
        this.spritePool = spritePool;
        this.spritesheetManager = spritesheetManager;

        this.layerObject = new PIXI.Container();
    }

    public getLayerObject = () => {
        return this.layerObject;
    };

    public handleLayerMessage = (data: ArrayBuffer) => {
        const incomingDataView = new DataView(data);

        // Parse the layer number
        const layer = incomingDataView.getUint8(0);
        const incomingSpriteCount = (data.byteLength - 1) / BYTES_PER_SHAPE;

        if (layer !== this.id) {
            throw new LayerParsingException(
                'I received the wrong layer id, ' +
                    'check that you are calling the right layer',
            );
        } else if (incomingSpriteCount !== Math.round(incomingSpriteCount)) {
            throw new LayerParsingException(
                'I received an invalid number of bytes, ' +
                    'the message is probably malformed',
            );
        }

        // Add/remove allocated sprites in layer
        const currSpriteCount = this.layerObject.children.length;
        if (incomingSpriteCount > currSpriteCount) {
            const newSprites = this.spritePool.allocateSprites(
                incomingSpriteCount - currSpriteCount,
            );
            this.layerObject.addChild(...newSprites);
        } else if (incomingSpriteCount < currSpriteCount) {
            // TODO
        }

        const sprites = this.layerObject.children as PIXI.Sprite[];
        sprites.forEach((sprite, index) => {
            // We start at 1 as the first byte of the message
            // contain the layer number
            const startPos = 1 + index * BYTES_PER_SHAPE;

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

            sprite.anchor.set(0.5, 0.5);
            sprite.texture = this.spritesheetManager.getTextureByID(textureId)!;
            sprite.x = x + width / 2;
            sprite.y = y + height / 2;
            sprite.width = width;
            sprite.height = height;
            sprite.rotation = rotation;
            sprite.alpha = opacity / 255;
            sprite.tint = ((red & 0xff) << 16) | ((green & 0xff) << 8) | (blue & 0xff);
        });
    };
}
