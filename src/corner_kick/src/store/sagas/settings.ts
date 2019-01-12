/**
 * This file specifies the settings saga
 */

import { put, spawn, takeEvery } from 'redux-saga/effects';

import { ISettingsState } from 'SRC/types';

import { hydrateSettings, set } from '../actions/settings';

const SETTINGS_KEY = 'settings';

let storedSettings = {};

/**
 * This function is called when the application first starts
 */
export default function* init() {
    yield spawn(startSettings);
}

/**
 * Fetches application settings stored in local storage and adds them to the
 * state
 */
function* startSettings() {
    const storedSettingsString = localStorage.getItem(SETTINGS_KEY);

    if (storedSettingsString !== null) {
        storedSettings = JSON.parse(storedSettingsString);
        yield put(hydrateSettings(storedSettings as ISettingsState));
    }

    // Begin listening to any action modifying the settings
    yield takeEvery('settings/SET', onSettingUpdate);
}

/**
 * Updates the settings stored in local storage to ensure they persist between
 * application runs
 */
function onSettingUpdate(action: ReturnType<typeof set>) {
    storedSettings = {
        ...storedSettings,
        [action.payload.id]: action.payload.value,
    };

    localStorage.setItem(SETTINGS_KEY, JSON.stringify(storedSettings));
}
