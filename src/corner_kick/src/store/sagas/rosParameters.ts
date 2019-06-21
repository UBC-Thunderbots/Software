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

    // We generate our param config object from the yaml description files
    params.keys().forEach((key) => {
        // We isolate the filename of the param as it is used by ROS to fetch
        // the current param value
        const filename = key.replace(/.*\//, '').replace(/\..*/, '');

        parseParam(rosParamSettings, `/${filename.toLowerCase()}`, params(key));
    });

    // We then fetch the current param value from ROS
    for (const key in rosParamSettings) {
        rosParamSettings[key].value = yield ROS.getParam(
            `${rosParamSettings[key].root}/${key}`,
        );
    }

    // Send the config to our state
    yield put(actions.rosParameters.hydrateROSParams(rosParamSettings));

    // Wait for param changes and send them to ROS
    yield takeLatest(
        getType(actions.rosParameters.setParam),
        ({ payload }: ReturnType<typeof actions.rosParameters.setParam>) => {
            setParam(payload.key, payload.value);
        },
    );
}

/**
 * Parses a JSON object representation of a param description YAML file into an
 * indexed flat tree
 */
const parseParam = (arrayOfParams: IROSParamState, file: string, params: any) => {
    Object.keys(params).forEach((key) => {
        // Check if current param is a leaf, defined by having a type attribute
        if (params[key]['type'] !== undefined) {
            // Add it to our flat tree
            arrayOfParams[key] = params[key];

            // Keep a reference to the filename this param comes from as we will
            // need it to access the param value
            arrayOfParams[key].root = file;
        } else {
            // If we did not arrive to a param leaf, we try again at the node
            parseParam(arrayOfParams, file, params[key]);
        }
    });
};

/**
 * Updates the param value in the ROS param server
 */
const setParam = (key: string, value: boolean | string) => {
    const config = {};

    // Behaviour will vary depending on variable type, we need to check it
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
