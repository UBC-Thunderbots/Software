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
        parseParam(rosParamSettings, `/${filename.toLowerCase()}`, params(key));
    });

    for (const key in rosParamSettings) {
        rosParamSettings[key].value = yield ROS.getParam(
            `${rosParamSettings[key].root}/${key}`,
        );
    }

    yield put(actions.rosParameters.hydrateROSParams(rosParamSettings));

    yield takeLatest(
        getType(actions.rosParameters.setParam),
        ({ payload }: ReturnType<typeof actions.rosParameters.setParam>) => {
            setParam(payload.key, payload.value);
        },
    );
}

const parseParam = (arrayOfParams: IROSParamState, file: string, params: any) => {
    Object.keys(params).forEach((key) => {
        if (params[key]['type'] !== undefined) {
            arrayOfParams[key] = params[key];
            arrayOfParams[key].root = file;
        } else {
            parseParam(arrayOfParams, file, params[key]);
        }
    });
};

const setParam = (key: string, value: boolean | string) => {
    const config = {};
    switch (typeof value) {
        case 'string':
            config['strs'] = [{ name: key, value }];
            break;
        case 'boolean':
            config['bools'] = [{ name: key, value }];
            break;
    }

    ROS.sendRequestToService(
        '/ai_control/set_parameters',
        'dynamic_reconfigure/Reconfigure',
        {
            config,
        },
    );
};
