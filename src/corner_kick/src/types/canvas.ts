/*
 * This file describes the datatypes used for the canvas
 */

/**
 * Represents a parsed layer received from the websocket
 */
export interface ILayerMessage {
    /**
     * The layer number
     */
    layer: number;

    /**
     * Sprite data
     */
    sprites: ISprite[];
}

/**
 * Represents a layer in the canvas
 */
export interface ILayer {
    /**
     * Name of the layer
     */
    name: string;

    /**
     * Specifies if the layer is visible in the canvas
     */
    visible: boolean;
}

/**
 * Represents a sprite in the canvas
 */
export interface ISprite {
    texture: number;
    x: number;
    y: number;
    width: number;
    height: number;
    rotation: number;
    opacity: number;
    red: number;
    green: number;
    blue: number;
}
