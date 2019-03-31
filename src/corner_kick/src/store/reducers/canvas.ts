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
        // Here, we add the layer to our layer dictionary and
        // append it in the layer display order (aka. the new layer is displayed
        // on top of old ones)
        case getType(canvas.addLayer):
            return {
                ...state,
                layers: { ...state.layers, [action.payload.id]: action.payload },
                layerOrder: [...state.layerOrder, action.payload.id],
            };
        // Here, we toggle the visibility of the layer specified by the action
        case getType(canvas.toggleLayerVisibility):
            const newVisibility = state.layers[action.payload.id].visible ? false : true;
            return {
                ...state,
                layers: {
                    ...state.layers,
                    [action.payload.id]: {
                        ...state.layers[action.payload.id],
                        visible: newVisibility,
                    },
                },
            };
        default:
            return state;
    }
};
