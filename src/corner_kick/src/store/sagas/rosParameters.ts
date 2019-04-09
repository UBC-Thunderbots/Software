/*
 * This file specifies the saga for ROSParams
 */
import { put, takeLatest } from 'redux-saga/effects';
import { IROSParamState } from 'SRC/types';
import { getType } from 'typesafe-actions';

import { actions } from '../actions';
import { hydrateROSParams } from '../actions/rosParameters';

const rosParamSettings: { [key: string]: any } = {};

export default function* init() {
    // Listen to start actions and start ROS Parameter
    yield takeLatest(getType(actions.ros.connected), startROSParameter);
}

/**
 * We subscribe to topic rosout to start receiving messages
 */
function* startROSParameter() {
    const params = require.context('SRC/utils/ros/params', false, /.*\.yaml/);

    params.keys().forEach((key) => {
        parseParam(rosParamSettings, params(key));
    });

    yield put(hydrateROSParams(rosParamSettings as IROSParamState));
}

const parseParam = (arrayOfParams: { [key: string]: any }, params: any) => {
    Object.keys(params).forEach((key) => {
        if (params[key]['type'] !== undefined) {
            arrayOfParams[key] = params[key];
        } else {
            parseParam(arrayOfParams, params[key]);
        }
    });
};
