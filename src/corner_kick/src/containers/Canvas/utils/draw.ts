/**
 * This file specifies functions to draw various types of shapes
 */

import {
    IArcShape,
    IEllipseShape,
    ILineShape,
    IPolyShape,
    IRectShape,
    IShape,
} from 'SRC/types';

/**
 * Sets the correct draw style (fill and stroke). All sprites are white to ensure
 * correct tinting accuracy
 * @param ctx an canvas rendering context
 * @param shape the shape description object
 */
export const setStyle = (ctx: CanvasRenderingContext2D, shape: IShape) => {
    ctx.fillStyle = shape.fill === undefined || shape.fill ? 'white' : 'transparent';
    ctx.strokeStyle = shape.stroke ? 'white' : 'transparent';
    ctx.lineWidth = shape.stroke_weight || 0;
};

/**
 * Draws a rectangle
 * @param ctx an canvas rendering context
 * @param shape the shape description object
 */
export const drawRect = (ctx: CanvasRenderingContext2D, shape: IRectShape) => {
    ctx.rect(shape.data[0], shape.data[1], shape.data[2], shape.data[3]);
};

/**
 * Draws an ellipse
 * @param ctx an canvas rendering context
 * @param shape the shape description object
 */
export const drawEllipse = (ctx: CanvasRenderingContext2D, shape: IEllipseShape) => {
    ctx.arc(shape.data[0], shape.data[1], shape.data[2], 0, 2 * Math.PI, false);
};

/**
 * Draws an arc
 * @param ctx an canvas rendering context
 * @param shape the shape description object
 */
export const drawArc = (ctx: CanvasRenderingContext2D, shape: IArcShape) => {
    ctx.arc(
        shape.data[0],
        shape.data[1],
        shape.data[2],
        shape.data[3],
        shape.data[4],
        false,
    );
};

/**
 * Draws a line
 * @param ctx an canvas rendering context
 * @param shape the shape description object
 */
export const drawLine = (ctx: CanvasRenderingContext2D, shape: ILineShape) => {
    ctx.moveTo(shape.data[0], shape.data[1]);
    ctx.lineTo(shape.data[2], shape.data[3]);
};

/**
 * Draws a poly. If the shape object does not contain enough coordinate sets, or
 * contains an invalid number of coordinates, the shape will not be drawn.
 * @param ctx an canvas rendering context
 * @param shape the shape description object
 */
export const drawPoly = (ctx: CanvasRenderingContext2D, shape: IPolyShape) => {
    if (shape.data.length % 2 === 0 && shape.data.length >= 6) {
        ctx.moveTo(shape.data[0], shape.data[1]);
        for (let i = 2; i < shape.data.length; i += 2) {
            ctx.lineTo(shape.data[i], shape.data[i + 1]);
        }
        ctx.closePath();
    }
};
