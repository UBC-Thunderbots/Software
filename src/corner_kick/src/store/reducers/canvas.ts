/*
 * This file specifies the Canvas reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { ICanvasState } from 'SRC/types';

import * as canvas from '../actions/canvas';

export type CanvasAction = ActionType<typeof canvas>;

const defaultState: ICanvasState = {
    layers: [],
};

export default (state: ICanvasState = defaultState, action: CanvasAction) => {
    switch (action.type) {
        case getType(canvas.addLayer):
            return {
                ...state,
                layers: [...state.layers, action.payload],
            };
        default:
            return state;
    }
};
