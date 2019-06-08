/*
 * This file combines all reducers to be consumed by the Redux store
 */

import { combineReducers } from 'redux';

import canvasReducer from './canvas';
import thunderbotsReducer from './thunderbots';
import rosReducer from './ros';
import rosParametersReducer from './rosParameters';

/**
 * Combines all reducers. This is what the Redux accepts when being
 * initialized
 */
export default combineReducers({
    canvas: canvasReducer,
    thunderbots: thunderbotsReducer,
    ros: rosReducer,
    rosParameters: rosParametersReducer,
});
