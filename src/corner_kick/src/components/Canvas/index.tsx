/**
 * This file defines the Canvas, an area that allows for custom shapes to be
 * drawn
 */

import * as React from 'react';

import { ILayer } from 'SRC/types';
import styled from 'SRC/utils/styled-components';

import { Layer } from './Layer';

const Wrapper = styled.svg`
    & * {
        fill: transparent;
        stroke: transparent;
    }
`;

interface ICanvasProps {
    /**
     * Starting x value for the Canvas (ie. the origin)
     * Defaults to 0
     */
    startX?: number;

    /**
     * Starting y value for the Canvas (ie. the origin)
     * Defaults to 0
     */
    startY?: number;

    /**
     * Width of the Canvas in world units
     */
    worldWidth: number;

    /**
     * Height of the Canvas in world units
     */
    worldHeight: number;

    /**
     * The layers to display in the canvas
     */
    layers: ILayer[];
}

/**
 * A Canvas allows for custom shapes to be drawn.
 *
 * Will take the full size of the container it is positioned in
 */
export const Canvas = (props: ICanvasProps) => {
    const { layers, startX, startY, worldWidth, worldHeight } = props;
    return (
        <Wrapper
            xmlns="http://www.w3.org/2000/svg"
            width="100%"
            height="100%"
            viewBox={[startX || 0, startY || 0, worldWidth, worldHeight].join(' ')}
        >
            {layers.map((layer) => (
                <Layer layer={layer} />
            ))}
        </Wrapper>
    );
};
