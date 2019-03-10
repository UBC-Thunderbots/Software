/**
 * This file contains the logic to parse layer data
 */

import { BYTES_PER_SHAPE } from 'SRC/constants/canvas';
import { IShape } from 'SRC/types';
import { ILayerMessage } from 'SRC/types/canvas';
import { MalformedShapeException } from 'SRC/utils/exceptions/malformedShapes';

/**
 * Parses an ArrayBuffer into a LayerMessage.
 * @throws MalformedShapeException if the ArrayBuffer contains an illegal byte count
 */
export const parseLayer = (data: ArrayBuffer): ILayerMessage => {
    const incomingDataView = new DataView(data);

    // Parse the layer number
    const layer = incomingDataView.getUint8(0);

    // Parse the flags for the layer
    const flags = incomingDataView.getUint8(1);
    const flagArray: boolean[] = new Array(8);
    for (let i = 0; i < 8; i++) {
        flagArray[i] = ((flags >> i) & 1) === 1;
    }

    const incomingSpriteCount = (data.byteLength - 2) / BYTES_PER_SHAPE;
    if (incomingSpriteCount !== Math.round(incomingSpriteCount)) {
        throw new MalformedShapeException(
            'The message from the server contains malformed data',
        );
    } else {
        // Parse each shape
        const shapes = new Array(incomingSpriteCount).fill(true).map(
            (_, index): IShape => {
                const startPos = 2 + index * BYTES_PER_SHAPE;
                return {
                    texture: incomingDataView.getUint8(startPos),
                    flags: incomingDataView.getUint8(startPos + 1),
                    x: incomingDataView.getInt16(startPos + 2),
                    y: incomingDataView.getInt16(startPos + 4),
                    width: incomingDataView.getInt16(startPos + 6),
                    height: incomingDataView.getInt16(startPos + 8),
                    rotation: incomingDataView.getInt16(startPos + 10),
                    opacity: incomingDataView.getUint8(startPos + 12),
                    red: incomingDataView.getUint8(startPos + 13),
                    green: incomingDataView.getUint8(startPos + 14),
                    blue: incomingDataView.getUint8(startPos + 15),
                };
            },
        );
        return {
            layer,
            flags: flagArray,
            shapes,
        };
    }
};
