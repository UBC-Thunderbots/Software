/**
 * Specifies the root component of the application
 */

import * as React from 'react';
import { Provider } from 'react-redux';

import { Console } from './components/portals/Console';
import { Main } from './components/portals/Main';
import { Sidebar } from './components/portals/Sidebar';
import { SidebarTitle } from './components/portals/SidebarTitle';
import { Theme } from './containers/mainUI/Theme';
import { createStore } from './store';

const store = createStore();

// Top-level React component in the application
export const App = () => (
    <Provider store={store}>
        <Theme>
            <Sidebar>This is the sidebar</Sidebar>
            <Main>This is main</Main>
            <SidebarTitle text="This is the sidebar title" />
            <Console>This is the console</Console>
        </Theme>
    </Provider>
);
