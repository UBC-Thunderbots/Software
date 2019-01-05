import { combineReducers } from 'redux';

import rosReducer, { ROSAction } from './ros';
import settingsReducer, { SettingsAction } from './settings';
import visualizerReducer from './visualizer';

export default combineReducers({
    ros: rosReducer,
    settings: settingsReducer,
    visualizer: visualizerReducer,
});

export type RootAction = ROSAction | SettingsAction;
