import { ActionType, getType } from 'typesafe-actions';

import * as settings from '../actions/settings';
import { ISettingsState } from '../state/settings';

export type SettingsAction = ActionType<typeof settings>;

const defaultState: ISettingsState = {
    fg_color: '#FFF',
    ros_refresh_rate: '5000',
    visualizer_refresh_rate: '32',
};

export default (state: ISettingsState = defaultState, action: SettingsAction) => {
    switch (action.type) {
        case getType(settings.hydrateSettings):
            return {
                ...state,
                ...action.payload.settings,
            };
        case getType(settings.set):
            return { ...state, [action.payload.id]: action.payload.value };
        default:
            return state;
    }
};
