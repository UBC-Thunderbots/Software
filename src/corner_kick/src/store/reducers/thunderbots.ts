/***
 * This file specifies the Thunderbots reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IThunderbotsState } from 'SRC/types';

import * as thunderbots from '../actions/thunderbots';

export type ThunderbotsAction = ActionType<typeof thunderbots>;

const defaultState: IThunderbotsState = {
    playType: 'Unknown',
    playName: 'Unknown',
    tactics: [],
};

/**
 * Reducer function for Thunderbots
 */
export default (state: IThunderbotsState = defaultState, action: ThunderbotsAction) => {
    switch (action.type) {
        // Update the state when we get new tactic info
        case getType(thunderbots.setPlayInformation):
            const { message } = action.payload;
            return {
                ...state,
                playType: message.play_type,
                playName: message.play_name,
                tactics: message.robot_tactic_assignment,
            };
        default:
            return state;
    }
};
