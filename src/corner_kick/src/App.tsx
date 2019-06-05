/*
 * Specifies the root component of the application
 */

import * as React from 'react';
import { Provider } from 'react-redux';

import * as ROS from 'SRC/utils/ros';
import { Portal, PortalLocation } from './components/Portal';
import { Logger } from './containers/Logger';
import { Visualizer } from './pages/Visualizer';
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
            <Visualizer />
            <Portal portalLocation={PortalLocation.CONSOLE}>
                <button onClick={startRunAi}>Start run_ai</button>
                <button onClick={stopRunAi}>Stop run_ai</button>
                <Logger />
            </Portal>
        </Theme>
    </Provider>
);

/**
 * Start run_ai
 */
function startRunAi() {
    ROS.sendRequestToService(
        '/ai_control/set_parameters',
        'dynamic_reconfigure/Reconfigure',
        {
            config: {
                bools: [{ name: 'run_ai', value: true }],
            },
        },
    );
    store.dispatch(actions.rosParameters.setRunAI(true));
}

/**
 * Stop run_ai
 */
function stopRunAi() {
    ROS.sendRequestToService(
        '/ai_control/set_parameters',
        'dynamic_reconfigure/Reconfigure',
        {
            config: {
                bools: [{ name: 'run_ai', value: false }],
            },
        },
    );
    store.dispatch(actions.rosParameters.setRunAI(false));
}
