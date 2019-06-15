/*
 * This file specifies the saga for ROSParams
 */

import { put, takeLatest } from 'redux-saga/effects';
import { IROSParamState } from 'SRC/types';
import { getType } from 'typesafe-actions';

import * as ROS from 'SRC/utils/ros';

import { actions } from '../actions';

export default function* init() {
    // Listen to start actions and start ROS Parameter
    yield takeLatest(getType(actions.ros.connected), startROSParameter);
}

/**
 * Hydrate ROS parameters state with default values
 */
function* startROSParameter() {
    const params = require.context('SRC/utils/ros/params', false, /.*\.yaml/);

    const rosParamSettings: { [key: string]: any } = {};

    params.keys().forEach((key) => {
        parseParam(rosParamSettings, params(key));
    });

    yield put(actions.rosParameters.hydrateROSParams(rosParamSettings as IROSParamState));

    yield takeLatest(
        getType(actions.rosParameters.setRunAI),
        ({ payload }: ReturnType<typeof actions.rosParameters.setRunAI>) => {
            setParam({
                bools: [{ name: 'run_ai', value: payload.value }],
            });
        },
    );
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

const setParam = (config: any) => {
    ROS.sendRequestToService(
        '/ai_control/set_parameters',
        'dynamic_reconfigure/Reconfigure',
        {
            config,
        },
    );
};
