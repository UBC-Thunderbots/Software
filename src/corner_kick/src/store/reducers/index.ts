/*
 * This file combines all reducers to be consumed by the Redux store
 */

import { combineReducers } from 'redux';

import consoleReducer from './console';
import rosReducer from './ros';
import rosParametersReducer from './rosParameters';

/**
 * Combines all reducers. This is what the Redux accepts when being
 * initialized
 */
export default combineReducers({
    console: consoleReducer,
    ros: rosReducer,
    rosParameters: rosParametersReducer,
});
