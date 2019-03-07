/*
 * This file combines all reducers to be consumed by the Redux store
 */

import { combineReducers } from 'redux';

import consoleReducer from './console';
import paramsReducer from './params';
import rosReducer from './ros';

/**
 * Combines all reducers. This is what the Redux accepts when being
 * initialized
 */
export default combineReducers({
    console: consoleReducer,
    params: paramsReducer,
    ros: rosReducer,
});
