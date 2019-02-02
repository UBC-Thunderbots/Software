/**
 * This file defines a Layer in the Canvas
 */

import * as React from 'react';

import { ILayer } from 'SRC/types';

import { Rect } from './shapes/Rect';

interface ILayerProps {
    layer: ILayer;
}

/**
 * A layer in the Canvas. Contains the shapes of the layer and allows
 * for visibility to be toggled.
 */
export const Layer = (props: ILayerProps) => {
    // We return null if the layer is not visible
    if (!props.layer.visible) {
        return null;
    }

    // We select the type of shape to return based on the type property of the
    // shape
    const shapesElements = props.layer.shapes.map((shape, index) => {
        switch (shape.type) {
            case 'rect':
                return <Rect key={`${props.layer.name}${index}`} shape={shape} />;
            default:
                return null;
        }
    });

    return <>{shapesElements}</>;
};
