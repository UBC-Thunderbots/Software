import { put, takeLatest } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import * as ros from '../actions/ros';

/*
 * This file specifies the saga for the Console
 */

export default function* init() {
    yield takeLatest(getType(ros.connected), startConsole);
}

function* startConsole() {
    yield put(ros.subscribeTopic('/rosout', 'rosgraph_msgs/Log'));
}
