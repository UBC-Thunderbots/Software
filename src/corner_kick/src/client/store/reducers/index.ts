import { combineReducers } from 'redux';

import loggerReducer from './logger';
import rosReducer, { ROSAction } from './ros';
import settingsReducer, { SettingsAction } from './settings';
import visualizerReducer from './visualizer';

export default combineReducers({
    logger: loggerReducer,
    ros: rosReducer,
    settings: settingsReducer,
    visualizer: visualizerReducer,
});

export type RootAction = ROSAction | SettingsAction;
