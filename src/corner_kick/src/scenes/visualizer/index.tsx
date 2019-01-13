import * as React from 'react';
import { Portal } from 'SRC/components/Portal';
import { SidebarPanels } from 'SRC/components/SidebarPanels';
import { Panel } from 'SRC/components/SidebarPanels/Panel';
import { Layers } from './Layers';

export class Visualizer extends React.Component {
    public render() {
        return (
            <>
                <SidebarPanels>
                    <Panel title="Layers">
                        <Layers
                            layers={[
                                {
                                    name: 'Friendly Robots',
                                    topic: 'friendly_robots',
                                    visible: true,
                                },
                                {
                                    name: 'Enemy Robots',
                                    topic: 'enemy_robots',
                                    visible: false,
                                },
                                {
                                    name: 'Ball',
                                    topic: 'ball',
                                    visible: true,
                                },
                                {
                                    name: 'Field',
                                    topic: 'field',
                                    visible: true,
                                },
                            ]}
                        />
                    </Panel>
                    <Panel title="AI Controls" />
                    <Panel title="Game Status" disabled={true} />
                </SidebarPanels>
                <Portal to="main">This is the visualizer</Portal>
            </>
        );
    }
}
