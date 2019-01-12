import { put, takeLatest } from 'redux-saga/effects';

import { subscribeTopic } from '../actions/ros';

export default function* init() {
    yield takeLatest('ros/CONNECTED', startLogger);
}

function* startLogger() {
    yield put(subscribeTopic('/rosout', 'rosgraph_msgs/Log'));
}
