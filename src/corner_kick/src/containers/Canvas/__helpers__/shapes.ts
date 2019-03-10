/**
 * This file contains helper functions for unit testing
 */

import * as Chance from 'chance';

import { BYTES_PER_SHAPE } from 'SRC/constants/canvas';
import { IShape } from 'SRC/types';

const chance = new Chance();

/**
 * Generates a random number of shapes for testing purposes
 * @param count Number of shapes to generate
 */
export const generateRandomShapes = (count: number): IShape[] => {
    return new Array(count).fill(true).map(() => {
        return {
            texture: chance.integer({ min: 0, max: 255 }),
            flags: chance.integer({ min: 0, max: 255 }),
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
export const generateLayerBuffer = (layer: number, flags: number, shapes: IShape[]) => {
    const arrayBuffer = new ArrayBuffer(2 + shapes.length * BYTES_PER_SHAPE);
    const dataView = new DataView(arrayBuffer);

    dataView.setUint8(0, layer);
    dataView.setUint8(1, flags);

    shapes.forEach((shape, index) => {
        const startPos = 2 + index * BYTES_PER_SHAPE;
        dataView.setUint8(startPos, shape.texture);
        dataView.setUint8(startPos + 1, shape.flags);
        dataView.setInt16(startPos + 2, shape.x);
        dataView.setInt16(startPos + 4, shape.y);
        dataView.setInt16(startPos + 6, shape.width);
        dataView.setInt16(startPos + 8, shape.height);
        dataView.setInt16(startPos + 10, shape.rotation);
        dataView.setUint8(startPos + 12, shape.opacity);
        dataView.setUint8(startPos + 13, shape.red);
        dataView.setUint8(startPos + 14, shape.green);
        dataView.setUint8(startPos + 15, shape.blue);
    });

    return arrayBuffer;
};
