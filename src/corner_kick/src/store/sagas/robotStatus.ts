/***
 * This file defines the saga for robot statuses
 */

import { channel } from 'redux-saga';
import { put, spawn, take, takeLatest } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import { TOPIC_ROBOT_STATUS, TOPIC_ROBOT_STATUS_TYPE } from 'SRC/constants';
import { IRobotStatus, IRobotStatusMessage } from 'SRC/types';
import * as ROS from 'SRC/utils/ros';

import { actions } from '../actions/index';

const statusChannel = channel();

export default function* init() {
    // Listen to start actions and start robot status
    yield takeLatest(getType(actions.ros.connected), startRobotStatusSaga);

    // Start listening to robot status messages
    yield spawn(listenToConsoleChannel);
}

/**
 * Take any messages received from robot status and push them as Redux actions
 */
function* listenToConsoleChannel() {
    while (true) {
        const action = yield take(statusChannel);
        yield put(action);
    }
}

let processedMessages: { [key: string]: IRobotStatus } = {};

/**
 * We subscribe to topic robot_status to start receiving messages
 */
function startRobotStatusSaga() {
    ROS.subscribeToROSTopic(
        TOPIC_ROBOT_STATUS,
        TOPIC_ROBOT_STATUS_TYPE,
        (message: IRobotStatusMessage) => {
            processedMessages = { ...processedMessages, ...processMessage(message) };
        },
        100,
    );

    // We update the state once a second
    setTimeout(updateTimestampAndPush, 1000);
}

function updateTimestampAndPush() {
    // Push current robot status to the state
    statusChannel.put(actions.status.updateRobotStatuses(processedMessages));

    // And increase timestamp by 1 (one second elapsed)
    Object.values(processedMessages).forEach((processedMessage) => {
        processedMessage.timestamp = processedMessage.timestamp + 1;
    });
    setTimeout(updateTimestampAndPush, 1000);
}

function processMessage(message: IRobotStatusMessage) {
    const processedMessage: { [key: string]: IRobotStatus } = {};
    message.robot_messages.forEach((robotMessage: string) => {
        const robotStatus: IRobotStatus = {
            message: robotMessage,
            robot: message.robot,
            timestamp: 0,
        };

        processedMessage[`${message.robot}: ${robotMessage}`] = robotStatus;
    });

    return processedMessage;
}
