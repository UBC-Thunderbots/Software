/**
 * This file contains helper functions for unit testing
 */

import * as Chance from 'chance';

import { BYTES_PER_SHAPE } from 'SRC/constants';
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
    const arrayBuffer = new ArrayBuffer(1 + shapes.length * BYTES_PER_SHAPE);
    const dataView = new DataView(arrayBuffer);

    dataView.setUint8(0, layer);

    shapes.forEach((shape, index) => {
        const startPos = 1 + index * BYTES_PER_SHAPE;
        dataView.setUint8(startPos, shape.texture);
        dataView.setInt16(startPos + 1, shape.x);
        dataView.setInt16(startPos + 3, shape.y);
        dataView.setInt16(startPos + 5, shape.width);
        dataView.setInt16(startPos + 7, shape.height);
        dataView.setInt16(startPos + 9, shape.rotation);
        dataView.setUint8(startPos + 11, shape.opacity);
        dataView.setUint8(startPos + 12, shape.red);
        dataView.setUint8(startPos + 13, shape.green);
        dataView.setUint8(startPos + 14, shape.blue);
    });

    return arrayBuffer;
};
