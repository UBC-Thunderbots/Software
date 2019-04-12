/***
 * This file defines some constants for the canvas
 */

import { ISpritesheet, ShapeType } from 'SRC/types';

export const LAYER_WEBSOCKET_ADDRESS = 'ws://localhost:9091';

/**
 * Number of bytes for each shape received from ROS.
 */
export const BYTES_PER_SHAPE = 15;

/**
 * The default spritesheet for the Canvas
 *
 * Contains two textures: a rectangle at id 0 and a circle at id 1
 */
export const SPRITESHEET: ISpritesheet = {
    dimensions: {
        w: 1024,
        h: 512,
    },
    frames: [
        {
            dimensions: {
                x: 0,
                y: 0,
                w: 500,
                h: 500,
            },
            shapes: [
                {
                    type: ShapeType.RECT,
                    data: [0, 0, 500, 500],
                },
            ],
        },
        {
            dimensions: {
                x: 512,
                y: 0,
                w: 500,
                h: 500,
            },
            shapes: [
                {
                    type: ShapeType.ELLIPSE,
                    data: [250, 250, 250, 250],
                },
            ],
        },
    ],
};
