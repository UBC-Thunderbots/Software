/*
 * This file combines all reducers to be consumed by the Redux store
 */

import { combineReducers } from 'redux';

import canvasReducer from './canvas';
import thunderbots from './thunderbots';
import rosReducer from './ros';

/**
 * Combines all reducers. This is what the Redux accepts when being
 * initialized
 */
export default combineReducers({
    canvas: canvasReducer,
    thunderbots: thunderbots,
    ros: rosReducer,
});
