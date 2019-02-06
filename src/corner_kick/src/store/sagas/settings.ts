/*
 * This file specifies the saga for Settings
 */
import { put, spawn, takeEvery } from 'redux-saga/effects';

import * as settings from '../actions/settings';

export default function* init() {
    yield spawn(startSettings);
}

/**
 * Hydrate settings upon starting
 */
function* startSettings() {
    // Retrieve settings from local storage
    const initializedSettingsBuffer = localStorage.getItem('settings');
    if (initializedSettingsBuffer != null) {
        const initializedSettings = JSON.parse(initializedSettingsBuffer);
        // Put hydrateSettings action from item
        yield put(settings.hydrateSettings(initializedSettings));
    }
    // Listen for updateSettings
    yield takeEvery('UPDATE_SETTINGS', updateSettings);
}

/**
 * Update settings by putting into local storage
 */
function updateSettings() {
    // Use localStorage.setItem
    // localStorage.setItem('settings', JSON.stringify(RETRIEVED_SETTINGS))
}
