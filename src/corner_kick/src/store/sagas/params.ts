/*
 * This file specifies the saga for Params
 */

import { channel } from 'redux-saga';
import { put, spawn, take, takeLatest } from 'redux-saga/effects';
import ROSLIB from 'roslib';
import { getType } from 'typesafe-actions';

import { actions } from '../actions';

let ros: ROSLIB.Ros | null = null;
let rosService: ROSLIB.Service | null = null;
let rosServiceRequest: ROSLIB.ServiceRequest | null = null;

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
        if (getType(action.type) === 'params_UPDATE_PARAMS') {
            // update the stored state
            rosParamsStored = {
                ...rosParamsStored,
                [action.payload.key]: action.payload.value,
            };
            // Create a new service to set parameters
            rosService = new ROSLIB.Service({
                name: '/params/set_parameters', // 'params' for current implementation
                ros: ros,
                serviceType: 'dynamic_reconfigure/Reconfigure',
            });

            // Create a new service request to specify parameters
            rosServiceRequest = new ROSLIB.ServiceRequest({
                config: {
                    doubles: [
                        { name: 'static_field_position_quality_x_offset', value: 10.0 },
                        { name: 'static_field_position_quality_y_offset', value: 0.1 },
                    ],
                },
            });

            rosService.callService(rosServiceRequest, (result) => {
                console.log(JSON.stringify(result, null, 1));
            });
        }
        yield put(action);
    }
}

/**
 * Read from ROS params hydrate state
 */
function startROSParams() {
    // Get params from ROS param server
    ros = new ROSLIB.Ros({});
    // Name will be interchangable, params for now
    rosParam = new ROSLIB.Param({ ros: ros, name: 'params' });
    rosParam.getParams((params) => {
        // add each param to object after parsing
        // for key and value
        rosParamsStored = {
            ...rosParamsStored,
            rosparams: params.value,
        };
    });
    actions.params.hydrateROSParams({ rosparams: rosParamsStored });
}
