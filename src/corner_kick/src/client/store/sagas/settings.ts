import { put, spawn, takeEvery } from 'redux-saga/effects';

import { ISettingsState } from 'SRC/types';

import { hydrateSettings, set } from '../actions/settings';

const SETTINGS_KEY = 'settings';

let storedSettings = {};

export default function* init() {
    yield spawn(startSettings);
}

function* startSettings() {
    const storedSettingsString = localStorage.getItem(SETTINGS_KEY);

    if (storedSettingsString !== null) {
        storedSettings = JSON.parse(storedSettingsString);
        yield put(hydrateSettings(storedSettings as ISettingsState));
    }

    yield takeEvery('settings/SET', onSettingUpdate);
}

function onSettingUpdate(action: ReturnType<typeof set>) {
    storedSettings = {
        ...storedSettings,
        [action.payload.id]: action.payload.value,
    };

    localStorage.setItem(SETTINGS_KEY, JSON.stringify(storedSettings));
}
