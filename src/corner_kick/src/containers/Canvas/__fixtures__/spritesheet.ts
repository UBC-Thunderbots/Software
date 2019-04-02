/**
 * This file describes a fixture used in Storybook
 */

import { ISpritesheet, ShapeType } from 'SRC/types';

/**
 * An example spritesheet
 *
 * Contains the following:
 * 1. A rectangle
 * 2. A circle
 * 3. An arc stroke, from 0 degrees to 180 degrees (clockwise)
 * 4. A triangle stroke
 * 5. A line from top left to bottom right of the screen
 * 6. A filled arc from -90 degrees to 180 degrees
 */
export const expectedSpritesheetFormat: ISpritesheet = {
    dimensions: {
        h: 1024,
        w: 1536,
    },
    frames: [
        {
            dimensions: {
                h: 500,
                w: 500,
                x: 0,
                y: 0,
            },
            shapes: [
                {
                    data: [0, 0, 500, 500],
                    type: ShapeType.RECT,
                },
            ],
        },
        {
            dimensions: {
                h: 500,
                w: 500,
                x: 512,
                y: 0,
            },
            shapes: [
                {
                    data: [250, 250, 250, 250],
                    type: ShapeType.ELLIPSE,
                },
            ],
        },
        {
            dimensions: {
                h: 500,
                w: 500,
                x: 512,
                y: 512,
            },
            shapes: [
                {
                    data: [10, 10, 490, 490],
                    stroke: true,
                    stroke_weight: 10,
                    type: ShapeType.LINE,
                },
            ],
        },
        {
            dimensions: {
                h: 500,
                w: 500,
                x: 0,
                y: 512,
            },
            shapes: [
                {
                    data: [10, 10, 490, 490, 10, 490],
                    fill: false,
                    stroke: true,
                    stroke_weight: 10,
                    type: ShapeType.POLY,
                },
            ],
        },
        {
            dimensions: {
                h: 500,
                w: 500,
                x: 1024,
                y: 0,
            },
            shapes: [
                {
                    data: [250, 250, 200, 0, Math.PI],
                    fill: false,
                    stroke: true,
                    stroke_weight: 10,
                    type: ShapeType.ARC,
                },
            ],
        },
        {
            dimensions: {
                h: 500,
                w: 500,
                x: 1024,
                y: 512,
            },
            shapes: [
                {
                    data: [250, 250, 200, -(1 / 2) * Math.PI, Math.PI],
                    type: ShapeType.ARC,
                },
            ],
        },
    ],
};
