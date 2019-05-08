/*
 * This file specifies the saga for ROSParams
 */

import { channel } from 'redux-saga';
import { put, spawn, take, takeLatest } from 'redux-saga/effects';
import { IROSParamState } from 'SRC/types';
import * as ROS from 'SRC/utils/ros';
import { getType } from 'typesafe-actions';

import { actions } from '../actions';

const rosParametersChannel = channel();

const rosParamSettings: { [key: string]: any } = {};

export default function* init() {
    // Listen to start actions and start ROS Parameter
    yield takeLatest(getType(actions.ros.connected), startROSParameter);

    // Start listening to Console messages
    yield spawn(listenToRosParametersChannel);
}

/**
 * Take any messages received from Ros Parameters and push them as Redux actions
 * Send service requests to properly write
 */
function* listenToRosParametersChannel() {
    while (true) {
        const action = yield take(rosParametersChannel);
        yield put(action);

        const serviceName = action.payload.name;
        const serviceValue = action.payload.value;

        // Access saga to find what type and reassign accordingly for config

        ROS.sendRequestToService(
            '/' + serviceName + '/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    double: [{ value: serviceValue }],
                },
            },
        );
    }
}

/**
 * We subscribe to topic rosout to start receiving messages
 */
function* startROSParameter() {
    const params = require.context('SRC/utils/ros/params', false, /.*\.yaml/);

    params.keys().forEach((key) => {
        parseParam(rosParamSettings, params(key));
    });

    yield put(actions.rosParameters.hydrateROSParams(rosParamSettings as IROSParamState));
}

const parseParam = (arrayOfParams: { [key: string]: any }, params: any) => {
    Object.keys(params).forEach((key) => {
        if (params[key]['type'] !== undefined) {
            arrayOfParams[key] = params[key];

            arrayOfParams[key]['value'] = params[key]['default'];
        } else {
            parseParam(arrayOfParams, params[key]);
        }
    });
};
