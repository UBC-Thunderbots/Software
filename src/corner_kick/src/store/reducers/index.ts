/**
 * This file combines all reducers to be consumed by the Redux store
 */

import { combineReducers } from 'redux';

import loggerReducer from './logger';
import rosReducer from './ros';
import settingsReducer from './settings';

/**
 * Combines all reducers. This is what the Redux accepts when being
 * initialized
 */
export default combineReducers({
    logger: loggerReducer,
    ros: rosReducer,
    settings: settingsReducer,
});
