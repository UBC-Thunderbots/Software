/*
 * This file defines the page for the visualizer
 */

import * as React from 'react';

import { Sidebar } from 'SRC/components/portals/Sidebar';

import { LayersPanel } from './LayersPanel';

/**
 * The Visualizer page. Allows for the visualization of the Thunderbots AI.
 */
export class Visualizer extends React.Component {
    public render() {
        return (
            <Sidebar>
                <LayersPanel
                    layers={[
                        {
                            id: 'friendly_robots',
                            name: 'Friendly Robots',
                            visible: true,
                        },
                        {
                            id: 'enemy_robots',
                            name: 'Enemy Robots',
                            visible: false,
                        },
                        {
                            id: 'ball',
                            name: 'Ball',
                            visible: true,
                        },
                        {
                            id: 'field',
                            name: 'Field',
                            visible: true,
                        },
                    ]}
                />
            </Sidebar>
        );
    }
}
