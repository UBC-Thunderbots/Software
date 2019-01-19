/**
 * This files specifies settings specific actions
 *
 * We are using the format specified here
 * @see https://github.com/piotrwitek/typesafe-actions#createaction
 */

import { createAction } from 'typesafe-actions';

import { ISettingsState } from 'SRC/types';

/**
 * Adds settings from local storage to the state
 */
export const hydrateSettings = createAction('settings_HYDRATE', (resolve) => {
    return (settings: ISettingsState) => resolve({ settings });
});

/**
 * Updates a settings entry
 */
export const set = createAction('settings_SET', (resolve) => {
    return (id: string, value: string) => resolve({ id, value });
});
