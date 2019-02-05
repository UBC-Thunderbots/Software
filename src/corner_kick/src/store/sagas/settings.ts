/*
 * This file specifies the saga for Settings
 */
import { put, spawn, takeLatest } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import * as settings from '../actions/settings';

export default function* init() {
    yield spawn(startSettings);
}

/**
 * We hydrate settings upon starting
 */
function startSettings() {
    // Use localStorage.getItem
}

function updateSettings() {
    // Use localStorage.setItem
}
