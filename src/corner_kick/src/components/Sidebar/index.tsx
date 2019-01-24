/**
 * This file defines a sidebar React component
 */

import * as React from 'react';

import { Portal, PortalLocation } from '../Portal';
import { IResizeablePanelsProps, ResizeablePanels } from './ResizeablePanels';

export { Panel } from './Panel';

/**
 * Adds a sidebar to the sidebar area of the application. The sidebar consists
 * of a series of resizeable panels, provided by the user of the sidebar.
 */
export const Sidebar = (props: IResizeablePanelsProps) => {
    const { children } = props;
    return (
        <Portal portalLocation={PortalLocation.sidebar}>
            <ResizeablePanels {...props}>{children}</ResizeablePanels>
        </Portal>
    );
};
