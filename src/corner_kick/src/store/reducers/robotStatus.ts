/*
 * This file specifies the robots status reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IRobotStatusState } from 'SRC/types';

import * as status from '../actions/robotStatus';

export type StatusAction = ActionType<typeof status>;

const defaultState: IRobotStatusState = {
    statuses: {},
};

/**
 * Reducer function for robot status
 */
export default (state: IRobotStatusState = defaultState, action: StatusAction) => {
    switch (action.type) {
        case getType(status.updateRobotStatuses):
            return {
                ...state,
                statuses: { ...state.statuses, ...action.payload.statuses },
            };
        default:
            return state;
    }
};
