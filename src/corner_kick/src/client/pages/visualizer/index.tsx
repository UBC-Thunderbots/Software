import * as React from 'react';

import { Main } from 'SRC/components/portals/Main';
import { Sidebar } from 'SRC/components/portals/Sidebar';
import { SidebarTitle } from 'SRC/components/portals/SidebarTitle';

import { VisualizerMain } from './VisualizerMain';

export class VisualizerPage extends React.Component {
    public render() {
        return (
            <>
                <SidebarTitle text="Visualizer" />
                <Sidebar>Hello World!</Sidebar>
                <Main>
                    <VisualizerMain />
                </Main>
            </>
        );
    }
}
