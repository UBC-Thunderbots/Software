/*
 * This file specifies the saga for Params
 */
/*
import { channel } from 'redux-saga';
import { call, put, spawn, takeEvery, takeLatest } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';
*/

import { channel } from 'redux-saga';
import { put, spawn, take, takeLatest } from 'redux-saga/effects';
import ROSLIB from 'roslib';
import { getType } from 'typesafe-actions';

import { TOPIC_ROSOUT, TOPIC_ROSOUT_TYPE } from '../../constants';
import { IRosoutMessage } from '../../types';

import { actions } from '../actions';
import ros from '../reducers/ros';

let rosbridge: ROSLIB.Ros | null = null;
let rosParam: ROSLIB.Param | null = null;
const paramChannel = channel();

let rosParamsStored = {};

export default function* init() {
    // Listen to start actions and start ROS params
    yield takeLatest(getType(actions.ros.connected), startROSParams);

    // Start listening to ROS param server
    yield spawn(listenToParamChannel);
}

/**
 * Take any updates from ROS param and push them as Redux actions
 */
function* listenToParamChannel() {
    while (true) {
        const action = yield take(paramChannel);
        if (getType(action.type) === 'param_UPDATE_PARAMS') {
            // update the stored state
            rosParamsStored = {
                ...rosParamsStored,
                [action.payload.key]: action.payload.value,
            };
            rosParam = new ROSLIB.Param({ ros: rosbridge, name: action.payload.key });
            rosParam.set(action.payload.value);
        }
        yield put(action);
    }
}

/**
 * Hydrate ROS param server state
 */
function startROSParams() {
    // Get params from ROS param server
    rosbridge = new ROSLIB.Ros({});
    rosParam = new ROSLIB.Param({ ros: rosbridge, name: 'testParam' });
    rosParam.getParams(function(params) {
        // add each param to object
        rosParamsStored = {
            ...rosParamsStored,
            [params.key]: params.value, // check these properties
        };
    });
    actions.params.hydrateROSParams({ rosparams: rosParamsStored });
}
/*
function* startROSParams() {
    const rosParamsStoredString = localStorage.getItem('rosparams');
    if (rosParamsStoredString !== null) {
        rosParamsStored = JSON.parse(rosParamsStoredString);
        yield put(params.hydrateROSParams(rosParamsStored));
    }
}

export function* updateROSParams(action: ReturnType<typeof params.updateROSParams>) {
    rosParamsStored = {
        ...rosParamsStored,
        [action.payload.key]: action.payload.value,
    };
    yield call(localStorage.setItem, 'rosparams', JSON.stringify(rosParamsStored));
}
*/
