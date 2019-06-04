/*
 * This file specifies the saga for the Console
 */
import { channel } from 'redux-saga';
import { put, spawn, take, takeLatest } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import { TOPIC_ROSOUT, TOPIC_ROSOUT_TYPE } from 'SRC/constants';
import { IRosoutMessage } from 'SRC/types';
import * as ROS from 'SRC/utils/ros';

import { actions } from '../actions';

const consoleChannel = channel();

export default function* init() {
    // Listen to start actions and start Console
    yield takeLatest(getType(actions.ros.connected), startConsole);

    // Start listening to Console messages
    yield spawn(listenToConsoleChannel);
}

/**
 * Take any messages received from Console and push them as Redux actions
 */
function* listenToConsoleChannel() {
    while (true) {
        const action = yield take(consoleChannel);
        yield put(action);
    }
}

/**
 * We subscribe to topic rosout to start receiving messages
 */
function startConsole() {
    ROS.subscribeToROSTopic(
        TOPIC_ROSOUT,
        TOPIC_ROSOUT_TYPE,
        (message: IRosoutMessage) => {
            consoleChannel.put(actions.console.newRosoutMessage(message));
        },
    );
}
