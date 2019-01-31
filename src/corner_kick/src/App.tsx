/*
 * Specifies the root component of the application
 */

import * as React from 'react';
import { Provider } from 'react-redux';

import { Portal, PortalLocation } from './components/Portal';
import { createStore } from './store';
import { Theme } from './style/Theme';

const store = createStore();

// Top-level React component in the application
// !!!! THIS ... IS ... NOT ... HTML !!!!
// IF YOU DIDN'T REALIZE THIS, PLEASE GO AND READ
// https://reactjs.org/docs/introducing-jsx.html
export const App = () => (
    <Provider store={store}>
        <Theme>
            <Portal portalLocation={PortalLocation.SIDEBAR}>This is the sidebar</Portal>
            <Portal portalLocation={PortalLocation.MAIN}>This is main</Portal>
            <Portal portalLocation={PortalLocation.CONSOLE}>This is the console</Portal>
        </Theme>
    </Provider>
);
