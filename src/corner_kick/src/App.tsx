/*
 * Specifies the root component of the application
 */

import * as React from 'react';
import { Provider } from 'react-redux';

import * as ROS from 'SRC/utils/ros';
import { Portal, PortalLocation } from './components/Portal';
import { Logger } from './containers/Logger';
import { createStore } from './store';
import { Theme } from './style/Theme';

import { actions } from './store/actions';

const store = createStore();

// Top-level React component in the application
// !!!! THIS ... IS ... NOT ... HTML !!!!
// IF YOU DIDN'T REALIZE THIS, PLEASE GO AND READ
// https://reactjs.org/docs/introducing-jsx.html
export const App = () => (
    <Provider store={store}>
        <Theme>
            <Portal portalLocation={PortalLocation.SIDEBAR}>
                <button onClick={dispatchBtnAction}>Set Value</button>
                <button onClick={testROS}>Test ROS</button>
            </Portal>
            <Portal portalLocation={PortalLocation.MAIN}>This is main</Portal>
            <Portal portalLocation={PortalLocation.CONSOLE}>
                <Logger />
            </Portal>
        </Theme>
    </Provider>
);

/* Testing the action changing state */
function dispatchBtnAction() {
    store.dispatch(actions.rosParameters.setRunAI(true));
}

function testROS() {
    ROS.sendRequestToService(
        '/ai_control/set_parameters',
        'dynamic_reconfigure/Reconfigure',
        {
            config: {
                bools: [{ name: 'run_ai', value: true }],
            },
        },
    );
    console.log('Pressed testROS');
}
