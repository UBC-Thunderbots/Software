import { ISpritesheet, ShapeType } from 'SRC/types';

/*
 * This file defines some constants for the canvas
 */

export const LAYER_WEBSOCKET_ADDRESS = 'ws://localhost:9091';

/**
 * Number of bytes for each shape received from ROS.
 */
export const BYTES_PER_SHAPE = 15;

export const SPRITESHEET: ISpritesheet = {
    dimensions: {
        w: 128,
        h: 64,
    },
    frames: [
        {
            dimensions: {
                x: 0,
                y: 0,
                w: 60,
                h: 60,
            },
            shapes: [
                {
                    type: ShapeType.RECT,
                    data: [0, 0, 60, 60],
                },
            ],
        },
        {
            dimensions: {
                x: 64,
                y: 0,
                w: 60,
                h: 60,
            },
            shapes: [
                {
                    type: ShapeType.ELLIPSE,
                    data: [30, 30, 30, 30],
                },
            ],
        },
    ],
};
