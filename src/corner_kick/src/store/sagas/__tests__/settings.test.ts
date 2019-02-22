import { expectSaga } from 'redux-saga-test-plan';
import * as settingsSaga from '../settings';

import * as settings from '../../actions/settings';

describe('updateSettings', () => {
    it('update the settings in local storage', () => {
        const mockAction = {
            payload: settings.updateSettings('testKey', 'testVal'),
        };

        return (
            expectSaga(settingsSaga.updateSettings, mockAction)
                .call(
                    localStorage.setItem,
                    'settings',
                    JSON.stringify({ testKey: 'testVal' }),
                )
                // Start the test. Returns a Promise.
                .run()
        );
    });
});
