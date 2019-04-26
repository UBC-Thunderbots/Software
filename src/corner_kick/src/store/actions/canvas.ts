/*
 * This file specifies Canvas specific action
 *
 * We are using the format specified here
 * @see https://github.com/piotrwitek/typesafe-actions#createaction
 */

import { createAction } from 'typesafe-actions';

/**
 * Action to indicate that a new layer was received by the Canvas.
 *
 * By default, layer is visible.
 */
export const addLayer = createAction('canvas_ADD_LAYER', (resolve) => {
    return (id: number) => resolve({ id, visible: true });
});

/**
 * Action to toggle the visibility of a particular layer
 */
export const toggleLayerVisibility = createAction(
    'canvas_TOGGLE_VISIBILITY',
    (resolve) => {
        return (id: number) => resolve({ id });
    },
);
