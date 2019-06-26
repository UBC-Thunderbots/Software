/*
 * This file combines all reducers to be consumed by the Redux store
 */

import { combineReducers } from 'redux';

import canvasReducer from './canvas';
import rosReducer from './ros';
import rosParametersReducer from './rosParameters';
import thunderbotsReducer from './thunderbots';
import robotStatusReducer from './robotStatus';

/**
 * Combines all reducers. This is what the Redux accepts when being
 * initialized
 */
export default combineReducers({
    canvas: canvasReducer,
    thunderbots: thunderbotsReducer,
    ros: rosReducer,
    rosParameters: rosParametersReducer,
    robotStatus: robotStatusReducer,
});
