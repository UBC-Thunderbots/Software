import * as React from 'react';

import { Portal } from '../Portal';
import { Panel } from './Panel';
import { ResizeablePanels } from './ResizeablePanels';

interface ISidebarProps {
    children: React.ReactElement<typeof Panel> | Array<React.ReactElement<typeof Panel>>;
}

export const Sidebar = (props: ISidebarProps) => {
    const { children } = props;
    return (
        <Portal to="sidebar">
            <ResizeablePanels>{children}</ResizeablePanels>
        </Portal>
    );
};
