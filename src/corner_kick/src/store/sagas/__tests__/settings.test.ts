import { expectSaga } from 'redux-saga-test-plan';

import { updateSettings } from '../../actions/settings';

describe('updateSettings', () => {
    it('should update settings', () => {
        const callback = jest.fn();

        return (
            expectSaga(updateSettings as any, 'testKey', 'testValue', callback)
                .call.like({ args: [callback] })
                // Start the test. Returns a Promise.
                .run()
        );
    });
});
