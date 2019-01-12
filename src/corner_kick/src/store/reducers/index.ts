import { combineReducers } from 'redux';

import loggerReducer from './logger';
import rosReducer, { ROSAction } from './ros';
import settingsReducer, { SettingsAction } from './settings';

export default combineReducers({
    logger: loggerReducer,
    ros: rosReducer,
    settings: settingsReducer,
});

export type RootAction = ROSAction | SettingsAction;
