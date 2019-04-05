/*
 * This file specifies the saga for ROSParams
 */
import { takeLatest } from 'redux-saga/effects';
import { getType } from 'typesafe-actions';

import { actions } from '../actions';

export default function* init() {
    // Listen to start actions and start ROS Parameter
    yield takeLatest(getType(actions.ros.connected), startROSParameter);
}

/**
 * We subscribe to topic rosout to start receiving messages
 */
function startROSParameter() {
    const params = require.context('SRC/utils/ros/params', false, /.*\.yaml/);

    const arrayOfParams: { [key: string]: any } = {};

    params.keys().forEach((key) => {
        parseParam(arrayOfParams, params(key));
    });

    console.log(arrayOfParams);
    // concatenate the arrays to one array
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
