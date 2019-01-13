import * as React from 'react';

import styled from 'SRC/utils/styled-components';

import { Portal } from '../Portal';
import { Panel } from './Panel';

const Wrapper = styled.div`
    width: 100%;
    height: 100%;

    display: flex;
    flex-flow: column nowrap;
`;

interface ISidebarPanelsProps {
    children: React.ReactElement<Panel> | Array<React.ReactElement<Panel>>;
}

export class SidebarPanels extends React.Component<ISidebarPanelsProps> {
    public render() {
        return (
            <Portal to="sidebar">
                <Wrapper>{this.props.children}</Wrapper>
            </Portal>
        );
    }
}
