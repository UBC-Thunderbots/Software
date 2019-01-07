import { createAction } from 'typesafe-actions';

import { ISettingsState } from 'SRC/types';

export const hydrateSettings = createAction('settings/HYDRATE', (resolve) => {
    return (settings: ISettingsState) => resolve({ settings });
});

export const set = createAction('settings/SET', (resolve) => {
    return (id: string, value: string) => resolve({ id, value });
});
