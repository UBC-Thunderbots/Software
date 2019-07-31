of*
 * Specifies the root component of the application
 */

import * as React from 'react';
import { Provider } from 'react-redux';

import { Visualizer } from './pages/Visualizer';
import { createStore } from './store';
import { Theme } from './style/Theme';

import { Logo } from './components/Logo';
import { Portal, PortalLocation } from './components/Portal';

const store = createStore();

// Top-level React component in the application
// !!!! THIS ... IS ... NOT ... HTML !!!!
// IF YOU DIDN'T REALIZE THIS, PLEASE GO AND READ
// https://reactjs.org/docs/introducing-jsx.html
export const App = () => (
    <Provider store={store}>
        <Theme>
            <Portal portalLocation={PortalLocation.SIDEBAR_TITLE}>
                <Logo />
            </Portal>
            <Visualizer />
        </Theme>
    </Provider>
);
