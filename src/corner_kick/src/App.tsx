import * as React from 'react';
import { Provider } from 'react-redux';
import { BrowserRouter as Router } from 'react-router-dom';

import { Console } from './components/portals/Console';
import { Page } from './components/portals/Page';
import { LoggerConsole } from './containers/consoles/logger';
import { Global } from './containers/mainUI/Global';
import { SettingsPage } from './containers/pages/settings';
import { createStore } from './store';

const store = createStore();

export const App = () => (
    <Provider store={store}>
        <Global>
            <Router>
                <>
                    <Page
                        text="Settings"
                        icon="settings"
                        path="/"
                        component={SettingsPage}
                    />
                </>
            </Router>
            <Console>
                <LoggerConsole />
            </Console>
        </Global>
    </Provider>
);
