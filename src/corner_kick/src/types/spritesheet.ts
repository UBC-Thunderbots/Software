/**
 * This file specifies types for spritesheet generation
 */

/**
 * The various types of shapes that can be drawn in a spritesheet
 */
export enum ShapeType {
    RECT = 'rect',
    ELLIPSE = 'ellipse',
    LINE = 'line',
    ARC = 'arc',
    POLY = 'poly',
}

/**
 * Description object for spritesheet. Used by
 * `SpritesheetManager` to generate a spritesheet.
 */
export interface ISpritesheet {
    /**
     * The frames contained in this spritesheet
     */
    frames: IFrame[];
    /**
     * The size of the spritesheet, make sure all the frames are contained
     * inside these dimensions
     */
    dimensions: {
        h: number;
        w: number;
    };
}

/**
 * Describes a single frame in a spritesheet, which may contain many shapes
 */
export interface IFrame {
    /**
     * The dimension of the frame, any shape drawn outside of these dimensions
     * will be cropped
     */
    dimensions: { x: number; y: number; w: number; h: number };
    /**
     * The shapes of this frame, drawn relative to the frame top-left corner
     */
    shapes: IShape[];
}

/**
 * Describes a shape. Used to generate a spritesheet.
 */
export interface IShape {
    /**
     * The type of shape
     */
    type: ShapeType;

    /**
     * The data associated with the shape
     *
     * **rect**:
     * - `data[0]`: x in px
     * - `data[1]`: y in px
     * - `data[2]`: width in px
     * - `data[3]`: height in px
     *
     * **ellipse**
     * - `data[0]`: cx in px
     * - `data[1]`: cy in px
     * - `data[2]`: radiusX in px
     * - `data[3]`: radiusY in px
     *
     * **arc**
     * - `data[0]`: cx in px
     * - `data[1]`: cy in px
     * - `data[2]`: radius in px
     * - `data[3]`: startAngle in rads
     * - `data[4]`: endAngle in rads
     *
     * **line**
     * - `data[0]`: x1 in px
     * - `data[1]`: y1 in px
     * - `data[2]`: x2 in px
     * - `data[3]`: y2 in px
     *
     * **poly**
     * - Contains pairs of x and y coordinates specified in px, describing
     * the points of the polygon. The first and last point of the polygon
     * will automatically be connected. Must have at least 3 coordinate sets in
     * order for the shape to be valid and be drawn.
     */
    data: number[];

    /**
     * Should the shape be filled?
     */
    fill?: boolean;

    /**
     * Should the shape be stroked?
     */
    stroke?: boolean;

    /**
     * The stroke weight of the shape
     */
    stroke_weight?: number;
}
