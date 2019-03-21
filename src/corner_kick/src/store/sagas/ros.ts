/*
 * This file specifies the saga for ROS
 */
import { call, put, takeLatest } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import * as ROS from 'SRC/utils/ros';

import { connected, error, start } from '../actions/ros';

/**
 * Function first called when the application first starts
 */
export default function* init() {
    // Listen to start actions and start ROS
    yield takeLatest(getType(start), startROS);

    // Start ROS
    yield put(start());
}

/**
 * Start ROS
 */
function* startROS() {
    try {
        yield call(ROS.connect);
        yield put(connected());
    } catch {
        yield put(error('Cannot connect to ROS'));
    }
}
