/***
 * This file describes Thunderbots related actions
 *
 * This includes action for storing tactic related data
 */

import { createAction } from 'typesafe-actions';
import { IPlayInfoMessage } from 'SRC/types';

/**
 * Called when new tactic information is received from the AI
 */
export const setPlayInformation = createAction('thunderbots_SET_PLAY_INFO', (resolve) => {
    return (message: IPlayInfoMessage) => resolve({ message });
});
