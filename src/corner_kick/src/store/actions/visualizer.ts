/**
 * This files specifies visualizer specific actions
 */

import { createAction } from 'typesafe-actions';

/**
 * Updates a layer's visibility
 */
export const changeVisibility = createAction(
    'visualizer_CHANGE_VISIBILITY',
    (resolve) => {
        return (topic: string, visibility: boolean) => resolve({ topic, visibility });
    },
);

/**
 * Updates a layer's order
 */
export const changeOrder = createAction('visualizer_CHANGE_ORDER', (resolve) => {
    return (prevIndex: number, newIndex: number) =>
        resolve({
            newIndex,
            prevIndex,
        });
});
