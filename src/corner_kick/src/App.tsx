import * as React from 'react';
import { Provider } from 'react-redux';

import { Visualizer } from 'SRC/scenes/visualizer';

import { Global } from './components/Global';
import { createStore } from './store';

const store = createStore();

// TODO: Add page navigation logic. See issue #216.
export const App = () => (
    <Provider store={store}>
        <Global>
            <Visualizer />
        </Global>
    </Provider>
);
