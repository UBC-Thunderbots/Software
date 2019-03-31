/*
 * This file specifies the Canvas reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { ICanvasState } from 'SRC/types';

import * as canvas from '../actions/canvas';

export type CanvasAction = ActionType<typeof canvas>;

const defaultState: ICanvasState = {
    layers: {},
    layerOrder: [],
};

export default (state: ICanvasState = defaultState, action: CanvasAction) => {
    switch (action.type) {
        case getType(canvas.addLayer):
            return {
                ...state,
                layers: { ...state.layers, [action.payload.id]: action.payload },
                layerOrder: [...state.layerOrder, action.payload.id],
            };
        case getType(canvas.toggleLayerVisibility):
            const newValue = state.layers[action.payload.id].visible ? false : true;
            return {
                ...state,
                layers: {
                    ...state.layers,
                    [action.payload.id]: {
                        ...state.layers[action.payload.id],
                        visible: newValue,
                    },
                },
            };
        default:
            return state;
    }
};
