/**
 * This file contains helper functions for unit testing
 */

import * as Chance from 'chance';

import { BYTES_PER_SHAPE } from 'SRC/constants';
import { ISprite } from 'SRC/types';

const chance = new Chance();

/**
 * Generates a random number of sprites for testing purposes
 * @param count Number of sprites to generate
 */
export const generateRandomSprites = (count: number): ISprite[] => {
    return new Array(count).fill(true).map(() => {
        return {
            texture: chance.integer({ min: 0, max: 255 }),
            x: chance.integer({ min: -32768, max: 32767 }),
            y: chance.integer({ min: -32768, max: 32767 }),
            width: chance.integer({ min: -32768, max: 32767 }),
            height: chance.integer({ min: -32768, max: 32767 }),
            rotation: chance.integer({ min: -32768, max: 32767 }),
            opacity: chance.integer({ min: 0, max: 255 }),
            red: chance.integer({ min: 0, max: 255 }),
            green: chance.integer({ min: 0, max: 255 }),
            blue: chance.integer({ min: 0, max: 255 }),
        };
    });
};

/**
 * Converts a layer into a binary ArrayBuffer. Used for testing purposes.
 * @param layer layer number
 * @param flags layer flags
 * @param shapes layer shapes
 */
export const generateLayerBuffer = (layer: number, sprites: ISprite[]) => {
    const arrayBuffer = new ArrayBuffer(1 + sprites.length * BYTES_PER_SHAPE);
    const dataView = new DataView(arrayBuffer);

    dataView.setUint8(0, layer);

    sprites.forEach((sprite, index) => {
        const startPos = 1 + index * BYTES_PER_SHAPE;
        dataView.setUint8(startPos, sprite.texture);
        dataView.setInt16(startPos + 1, sprite.x);
        dataView.setInt16(startPos + 3, sprite.y);
        dataView.setInt16(startPos + 5, sprite.width);
        dataView.setInt16(startPos + 7, sprite.height);
        dataView.setInt16(startPos + 9, sprite.rotation);
        dataView.setUint8(startPos + 11, sprite.opacity);
        dataView.setUint8(startPos + 12, sprite.red);
        dataView.setUint8(startPos + 13, sprite.green);
        dataView.setUint8(startPos + 14, sprite.blue);
    });

    return arrayBuffer;
};
