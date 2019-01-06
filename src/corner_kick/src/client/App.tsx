import * as React from 'react';
import { Provider } from 'react-redux';
import { BrowserRouter as Router } from 'react-router-dom';

import { Global } from './Global';
import { createStore } from './store';

import { Console } from './components/portals/Console';
import { Page } from './components/portals/Page';

import { ROSPage } from './pages/ros';
import { SettingsPage } from './pages/settings';
import { VisualizerPage } from './pages/visualizer';

import { LoggerConsole } from './consoles/logger';

const store = createStore();

export const App = () => (
    <Router>
        <Provider store={store}>
            <Global />
            <Page
                text="Visualizer"
                icon="widgets"
                path="/visualizer"
                component={VisualizerPage}
            />
            <Page text="ROS" icon="group_work" path="/ros" component={ROSPage} />
            <Page
                text="Settings"
                icon="settings"
                path="/settings"
                component={SettingsPage}
            />
            <Console>
                <LoggerConsole />
            </Console>
        </Provider>
    </Router>
);
