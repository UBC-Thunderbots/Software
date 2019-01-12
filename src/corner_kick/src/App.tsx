import * as React from 'react';
import { Provider } from 'react-redux';

import { Console } from './components/portals/Console';
import { Main } from './components/portals/Main';
import { Sidebar } from './components/portals/Sidebar';
import { SidebarTitle } from './components/portals/SidebarTitle';
import { Global } from './containers/mainUI/Global';
import { createStore } from './store';

const store = createStore();

export const App = () => (
    <Provider store={store}>
        <Global>
            <Sidebar>This is the sidebar</Sidebar>
            <Main>This is main</Main>
            <SidebarTitle text="This is the sidebar title" />
            <Console>This is the console</Console>
        </Global>
    </Provider>
);
