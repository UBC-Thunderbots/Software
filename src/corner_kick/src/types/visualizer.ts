/*
 * This file describes the datatypes used for the visualizer
 */

/**
 * Represents a layer message received from ROS
 */
export interface ILayerMessage {
    /**
     * Name of the layer
     */
    name: string;

    /**
     * Shapes associated with the layer
     */
    shapes: IShape[];
}

/**
 * Used by the visualizer to add additional properties to the visualizer
 */
export interface ILayer extends ILayerMessage {
    /**
     * Specifies if the layer is visible in the Canvas
     */
    visible: boolean;
}

/**
 * Represents a shape in the visualizer
 */
export interface IShape {
    /**
     * Specifies the type of shape to display
     */
    type: string;

    /**
     * The data associated with the shape
     */
    data: number[];

    /**
     * The fill color of the shape
     */
    fill?: string;

    /**
     * The stroke of the shape
     */
    stroke?: string;

    /**
     * The stroke weight of the shape
     */
    stroke_weight?: number;
}
