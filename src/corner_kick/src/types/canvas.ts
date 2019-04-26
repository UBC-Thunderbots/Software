/*
 * This file describes the datatypes used for the canvas
 */

/**
 * Represent a layer message received from ROS
 */
export interface ILayerMessage {
    /**
     * Contains sprite data. Encoded as a base64 string.
     */
    data: string;
}

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

export interface ISprite {
    textureID: number;
    x: number;
    y: number;
    width: number;
    height: number;
    rotation: number;
    opacity: number;
    tint: number;
}
