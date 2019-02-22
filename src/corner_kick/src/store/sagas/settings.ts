/*
 * This file specifies the saga for Settings
 */
import { call, put, spawn, takeEvery } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import * as settings from '../actions/settings';

export default function* init() {
    yield spawn(startSettings);
    // Listen for updateSettings
    yield takeEvery(getType(settings.updateSettings), updateSettings);
}

let settingsStored = {};

/**
 * Hydrate settings upon starting
 */
function* startSettings() {
    // Retrieve settings from local storage
    const settingsStoredBuffer = localStorage.getItem('settings');
    if (settingsStoredBuffer !== null) {
        settingsStored = JSON.parse(settingsStoredBuffer);
        // Put hydrateSettings action from item
        yield put(settings.hydrateSettings(settingsStored));
    }
}

/**
 * Update settings by putting into local storage
 */
export function* updateSettings(action: ReturnType<typeof settings.updateSettings>) {
    // Use localStorage.setItem
    settingsStored = {
        ...settingsStored,
        [action.payload.key]: action.payload.value,
    };
    yield call(localStorage.setItem, 'settings', JSON.stringify(settingsStored));
}
