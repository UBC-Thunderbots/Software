/**
 * This file is for testing layer data parsing
 */

import { BYTES_PER_SHAPE } from 'SRC/constants/canvas';
import { MalformedShapeException } from 'SRC/utils/exceptions/malformedShapes';

import { generateLayerBuffer, generateRandomShapes } from '../__helpers__/shapes';
import { parseLayer } from '../utils/parseLayer';

describe('parseLayer', () => {
    describe('when we process layer data from the websocket', () => {
        it('correctly processes the message', () => {
            const layer = 2;
            const flags = 1;
            const shapeCount = 100;

            const shapes = generateRandomShapes(shapeCount);

            const arrayBuffer = generateLayerBuffer(layer, flags, shapes);

            const incomingSpriteCount = (arrayBuffer.byteLength - 2) / BYTES_PER_SHAPE;
            expect(incomingSpriteCount).toEqual(shapeCount);

            const result = parseLayer(arrayBuffer);

            expect(result.layer).toEqual(layer);
            expect(result.flags).toEqual([
                true,
                false,
                false,
                false,
                false,
                false,
                false,
                false,
            ]);
            expect(result.shapes.length).toEqual(shapeCount);
            expect(result.shapes).toStrictEqual(shapes);
        });

        it('detects when we send a malformed layer message', () => {
            const arrayBuffer = new ArrayBuffer(2 + BYTES_PER_SHAPE + 1);

            expect(() => parseLayer(arrayBuffer)).toThrow(MalformedShapeException);
        });
    });
});
