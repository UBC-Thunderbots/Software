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

let rosParamsStored = {};

/**
 * Hydrate ROS params upon starting
 */
function* startROSParams() {
    // Retrieve settings from ROS param server
    const rosParamsStoredString = localStorage.getItem('rosparams');
    if (rosParamsStoredString !== null) {
        rosParamsStored = JSON.parse(rosParamsStoredString);
        // Put hydrateROSParams action from item
        yield put(params.hydrateROSParams(rosParamsStored));
    }
}

/**
 * Update settings by putting into local storage
 */
export function* updateROSParams(action: ReturnType<typeof params.updateROSParams>) {
    rosParamsStored = {
        ...rosParamsStored,
        [action.payload.key]: action.payload.value,
    };
    yield call(localStorage.setItem, 'rosparams', JSON.stringify(rosParamsStored));
}
