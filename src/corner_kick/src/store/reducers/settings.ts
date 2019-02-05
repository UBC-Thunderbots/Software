/*
 * This file specifies the Settings reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IPersistDataState } from 'SRC/types';

import * as settings from '../actions/settings';
export type SettingsAction = ActionType<typeof settings>;

const defaultState: IPersistDataState = {
    testKey: 'testValue',
};

/**
 * Reducer function for Settings
 */
export default (state: IPersistDataState = defaultState, action: SettingsAction) => {
    switch (action.type) {
        // Read key-value pair into state
        case getType(settings.hydrateSettings):
            return {
                ...state,
                ...action.payload.settings,
            };

        // Push key-value pair to state
        case getType(settings.updateSettings):
            return {
                ...state,
                [action.payload.key]: action.payload.value,
            };

        default:
            return state;
    }
};
