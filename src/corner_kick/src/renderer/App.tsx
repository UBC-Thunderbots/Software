/**
 * @fileoverview Defines the main page of the application
 */

import * as React from 'react';
import { Provider } from 'unstated';

import { PaneLayout } from './components/ui/PaneLayout';
import { Logger } from './modules/Logger';
import { Visualizer } from './modules/Visualizer';

/**
 * @description The main page of our application.
 */
export default class App extends React.Component {
    public render() {
        return (
            <Provider>
                <PaneLayout top={<Visualizer />} bottom={<Logger />} />
            </Provider>
        );
    }
}
