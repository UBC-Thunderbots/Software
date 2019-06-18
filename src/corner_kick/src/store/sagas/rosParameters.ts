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

    const rosParamSettings: IROSParamState = {};

    params.keys().forEach((key) => {
        const filename = key.replace(/.*\//, '').replace(/\..*/, '');
        parseParam(rosParamSettings, `/${filename}`, params(key));
    });

    for (const key in rosParamSettings) {
        rosParamSettings[key].value = yield ROS.getParam(rosParamSettings[key].fullPath);
    }

    yield put(actions.rosParameters.hydrateROSParams(rosParamSettings));

    yield takeLatest(
        getType(actions.rosParameters.setBooleanParam),
        ({ payload }: ReturnType<typeof actions.rosParameters.setBooleanParam>) => {
            setBooleanParam(payload.key, payload.value);
        },
    );
}

const parseParam = (arrayOfParams: IROSParamState, file: string, params: any) => {
    Object.keys(params).forEach((key) => {
        if (params[key]['type'] !== undefined) {
            arrayOfParams[key] = params[key];
            arrayOfParams[key].fullPath = `${file}/${key.toLowerCase()}`;
        } else {
            parseParam(arrayOfParams, file, params[key]);
        }
    });
};

const setBooleanParam = (key: string, value: boolean) => {
    ROS.sendRequestToService(
        '/ai_control/set_parameters',
        'dynamic_reconfigure/Reconfigure',
        {
            config: {
                bools: [{ name: key, value }],
            },
        },
    );
};
