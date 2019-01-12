/**
 * This files specifies settings specific actions
 */

import { createAction } from 'typesafe-actions';

import { ISettingsState } from 'SRC/types';

/**
 * Adds settings from local storage to the state
 */
export const hydrateSettings = createAction('settings/HYDRATE', (resolve) => {
    return (settings: ISettingsState) => resolve({ settings });
});

/**
 * Updates a settings entry
 */
export const set = createAction('settings/SET', (resolve) => {
    return (id: string, value: string) => resolve({ id, value });
});
