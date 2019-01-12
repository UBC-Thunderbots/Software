import { put, takeLatest } from 'redux-saga/effects';

import { subscribeTopic } from '../actions/ros';

export default function* init() {
    yield takeLatest('ros/CONNECTED', startVisualizer);
}

function* startVisualizer() {
    yield put(subscribeTopic('/backend/field', 'thunderbots_msgs/Field'));
    yield put(subscribeTopic('/backend/ball', 'thunderbots_msgs/Ball'));
}
