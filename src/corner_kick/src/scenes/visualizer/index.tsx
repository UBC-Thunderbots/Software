import * as React from 'react';
import { Portal } from 'SRC/components/Portal';
import { SidebarPanels } from 'SRC/components/SidebarPanels';
import { Panel } from 'SRC/components/SidebarPanels/Panel';

export class Visualizer extends React.Component {
    public render() {
        return (
            <>
                <SidebarPanels>
                    <Panel title="Layers">This is layers</Panel>
                    <Panel title="AI Controls">This is ai controls</Panel>
                    <Panel title="Game Status">This is game status</Panel>
                </SidebarPanels>
                <Portal to="main">This is the visualizer</Portal>
            </>
        );
    }
}
