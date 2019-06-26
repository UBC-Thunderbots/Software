/*
 * This file specifies the saga for Thunderbots related logic
 *
 * This includes listening ROS for tactic data
 */
import { channel } from 'redux-saga';
import { put, spawn, take, takeLatest } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import { TOPIC_PLAY_INFO, TOPIC_PLAY_INFO_TYPE } from 'SRC/constants';
import { IPlayInfoMessage } from 'SRC/types';
import * as ROS from 'SRC/utils/ros';

import { actions } from '../actions';

const thunderbotsChannel = channel();

export default function* init() {
    // Listen to start actions and start the Thunderbots saga
    yield takeLatest(getType(actions.ros.connected), startThunderbotsSaga);

    // Start listening to Thunderbots related ROS messages
    yield spawn(listenToThunderbotsChannel);
}

/**
 * Take any messages received from Thunderbots messages
 * and push them as Redux actions
 */
function* listenToThunderbotsChannel() {
    while (true) {
        const action = yield take(thunderbotsChannel);
        yield put(action);
    }
}

/**
 * Subscribes to various Thunderbots topic, including tactic messages
 */
function startThunderbotsSaga() {
    // Listen to tactic data from the AI
    ROS.subscribeToROSTopic(
        TOPIC_PLAY_INFO,
        TOPIC_PLAY_INFO_TYPE,
        (message: IPlayInfoMessage) => {
            thunderbotsChannel.put(actions.thunderbots.setPlayInformation(message));
        },
        1000,
    );
}
