/*
 * This file specifies the Settings specific action
 *
 * We are using the format specified here
 * @see https://github.com/piotrwitek/typesafe-actions#createaction
 */

import { IPersistDataState } from 'SRC/types';
import { createAction } from 'typesafe-actions';

/**
 * Hydrate data to state
 */
export const hydrateSettings = createAction('HYDRATE_SETTINGS', (resolve) => {
    return (settings: IPersistDataState) => resolve({ settings });
});

/**
 * Send data to persist (currently only for strings key-value pairs)
 */
export const updateSettings = createAction('UPDATE_SETTINGS', (resolve) => {
    return (key: string, value: string) => resolve({ key, value });
});
