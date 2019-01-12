/**
 * This file specifies the saga for the Logger.
 */

import { put, takeLatest } from 'redux-saga/effects';

import { subscribeTopic } from '../actions/ros';

/**
 * Function called when the application starts
 */
export default function* init() {
    // We wait until we are connected to ROS before starting
    // the logger
    yield takeLatest('ros/CONNECTED', startLogger);
}

/**
 * Starts the Logger saga. Currently, this saga registers to the
 * /rosout topic and exits
 */
function* startLogger() {
    yield put(subscribeTopic('/rosout', 'rosgraph_msgs/Log'));
}
