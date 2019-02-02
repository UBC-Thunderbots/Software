/**
 * This file specifies the visualizer reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IVisualizerState } from 'SRC/types';

import * as visualizer from '../actions/visualizer';

import { ROSAction } from './ros';

export type VisualizerActions = ActionType<typeof visualizer>;

const defaultState: IVisualizerState = {
    layerOrder: [],
    layers: {},
};

export default (
    state: IVisualizerState = defaultState,
    action: VisualizerActions | ROSAction,
) => {
    switch (action.type) {
        case getType(visualizer.changeVisibility): {
            const { topic, visibility } = action.payload;

            return {
                ...state,
                layers: {
                    ...state.layers,
                    [topic]: {
                        ...state.layers[topic],
                        visible: visibility,
                    },
                },
            };
        }
        case getType(visualizer.changeOrder): {
            const { prevIndex, newIndex } = action.payload;

            // We cannot mutate the input state so we create a new array
            const layerOrder = new Array(...state.layerOrder);

            const temp = layerOrder.splice(prevIndex, 1);

            return {
                ...state,
                layerOrder: [
                    ...layerOrder.slice(0, newIndex),
                    temp,
                    ...layerOrder.slice(newIndex),
                ],
            };
        }
        default:
            return state;
    }
};
