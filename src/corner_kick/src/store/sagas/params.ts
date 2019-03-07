/*
 * This file specifies the saga for Params
 */
import { call, put, spawn, takeEvery } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import * as params from '../actions/params';

export default function* init() {
    yield spawn(startROSParams);
    // Listen for updateRosParams
    yield takeEvery(getType(params.updateROSParams), updateROSParams);
}

let settingsStored = {};

/**
 * Hydrate ROS params upon starting
 */
function* startROSParams() {
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
export function* updateROSParams(action: ReturnType<typeof settings.updateSettings>) {
    // Use localStorage.setItem
    settingsStored = {
        ...settingsStored,
        [action.payload.key]: action.payload.value,
    };
    yield call(localStorage.setItem, 'settings', JSON.stringify(settingsStored));
}
