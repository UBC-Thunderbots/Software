import * as rosActions from './ros';

import { ROSAction } from '../reducers/ros';

export const actions = {
    ros: rosActions,
};

// We combine all action types for convenient access throughout the application
export type RootAction = ROSAction;
