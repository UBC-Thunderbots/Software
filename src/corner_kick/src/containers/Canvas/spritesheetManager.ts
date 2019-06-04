/**
 * This file is responsible for generating the spritesheet used in the
 * Canvas.
 */

import * as PIXI from 'pixi.js';

import { ISpritesheet, ShapeType } from 'SRC/types';

import * as draw from './utils/draw';

/**
 * Generates a spritesheet from an object
 * @param spritesheet an object describing the spritesheet
 * @returns a Canvas object with the spritesheet generated
 */
const generateSpritesheet = (spritesheet: ISpritesheet) => {
    // Create a canvas and give it the dimension of the spritesheet
    const canvas = document.createElement('canvas');
    canvas.width = spritesheet.dimensions.w;
    canvas.height = spritesheet.dimensions.h;

    const ctx = canvas.getContext('2d')!;

    // Generate each frame
    spritesheet.frames.forEach((frame) => {
        // We will create an OffscreenCanvas for each frame and place it
        // in the primary canvas
        const frameCanvas = document.createElement('canvas');
        frameCanvas.width = frame.dimensions.w;
        frameCanvas.height = frame.dimensions.h;
        const frameCtx = frameCanvas.getContext('2d')!;

        // Draw each shape in the frame onto the frame canvas
        frame.shapes.forEach((shape) => {
            draw.setStyle(frameCtx, shape);
            frameCtx.beginPath();
            switch (shape.type) {
                case ShapeType.RECT:
                    draw.drawRect(frameCtx, shape);
                    break;
                case ShapeType.ELLIPSE:
                    draw.drawEllipse(frameCtx, shape);
                    break;
                case ShapeType.ARC:
                    draw.drawArc(frameCtx, shape);
                    break;
                case ShapeType.LINE:
                    draw.drawLine(frameCtx, shape);
                    break;
                case ShapeType.POLY:
                    draw.drawPoly(frameCtx, shape);
                    break;
            }
            frameCtx.fill();
            frameCtx.stroke();
        });

        // Draw the frame canvas onto the primary canvas at the correct position
        ctx.drawImage(
            frameCanvas as HTMLCanvasElement,
            frame.dimensions.x,
            frame.dimensions.y,
        );
    });

    return canvas;
};

/**
 * Generates and manages the texture of a spritesheet. Supports rect, ellipse,
 * line, arc, and poly shapes.
 */
export class SpritesheetManager {
    private template: ISpritesheet;

    private canvas: HTMLCanvasElement;
    private baseTexture: PIXI.BaseTexture;

    private textureDictionary: PIXI.Texture[];

    /**
     * Creates a new SpritesheetGenerator and generate the necessary textures
     * based on the provided spritesheet descriptor object
     * @param template an object describing a spritesheet
     */
    constructor(template: ISpritesheet) {
        this.template = template;

        // Generate the spritesheet base texture
        this.canvas = generateSpritesheet(this.template);
        this.baseTexture = new PIXI.BaseTexture(this.canvas as HTMLCanvasElement);

        // And create each texture by referencing the base texture
        this.textureDictionary = this.template.frames.map((frame) => {
            return new PIXI.Texture(
                this.baseTexture,
                new PIXI.Rectangle(
                    frame.dimensions.x,
                    frame.dimensions.y,
                    frame.dimensions.w,
                    frame.dimensions.h,
                ),
            );
        });
    }

    /**
     * Retrieve a texture by its id. This method should be called
     * to retrieve the correct texture from the number sent from the
     * layer websocket
     * @returns a texture or null if id is invalid
     */
    public getTextureByID = (id: number) => {
        // Check that the id is in the correct range
        if (id >= 0 && id <= this.textureDictionary.length) {
            // This is a special ID refering to a white rectangle.
            if (id === 0) {
                return PIXI.Texture.WHITE;
            } else {
                // All other ids start at one so we map them to our array
                return this.textureDictionary[id - 1];
            }
        } else {
            return null;
        }
    };

    /**
     * Generate an image of the spritesheet. Useful to debug the result of
     * the generation process.
     */
    public getSpritesheetPNG = () => {
        return this.canvas.toDataURL();
    };
}
