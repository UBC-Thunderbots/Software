/**
 * This file specifies the settings reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { defaultSettings } from 'SRC/constants';
import { ISettingsState } from 'SRC/types';

import * as settings from '../actions/settings';

export type SettingsAction = ActionType<typeof settings>;

export default (state: ISettingsState = defaultSettings, action: SettingsAction) => {
    switch (action.type) {
        // Hydrate the settings when we get a hydrate action (received settings from
        // local storage)
        case getType(settings.hydrateSettings):
            return {
                ...state,
                ...action.payload.settings,
            };

        // Update a settings entry
        case getType(settings.set):
            return { ...state, [action.payload.id]: action.payload.value };
        default:
            return state;
    }
};
