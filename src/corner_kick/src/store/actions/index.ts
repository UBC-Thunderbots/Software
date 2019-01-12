import * as rosActions from './ros';
import * as settingsActions from './settings';

import { ROSAction } from '../reducers/ros';
import { SettingsAction } from '../reducers/settings';

export const actions = {
    ros: rosActions,
    settings: settingsActions,
};

// We combine all action types for convenient access throughout the application
export type RootAction = ROSAction | SettingsAction;
