/*
 * This file specifies Canvas specific action
 *
 * We are using the format specified here
 * @see https://github.com/piotrwitek/typesafe-actions#createaction
 */

import { createAction } from 'typesafe-actions';

export const addLayer = createAction('canvas_ADD_LAYER', (resolve) => {
    return (id: number) => resolve({ id, visibility: true });
});
