/*
 * This file describes the datatypes used for the canvas
 */

/**
 * Represents a layer in the canvas
 */
export interface ILayer {
    /**
     * ID of the layer
     */
    id: number;

    /**
     * Specifies if the layer is visible in the canvas
     */
    visible: boolean;
}
