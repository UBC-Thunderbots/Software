/*
 * This file defines a React component which adds other elements
 * in the main area of the application
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';

import { UnknownPortalException } from 'SRC/utils/exceptions/unknownPortal';

/**
 * The area where we wish to portal content to
 *
 * These areas include:
 * -   **main:** Contains the primary focus of the page
 * -   **sidebarTitle:** Contains the page title and additional options
 *     specific to the page
 * -   **sidebar:** Contains additional controls or information relevant
 *     to the current page
 * -   **sidebarControl:** Contains navigation controls to switch between pages
 * -   **console:** Contains views visible across the application
 * -   **footer(Left|Right):** Contains simple breadcrumbs of information
 *     relevant to the application as a whole or to the current page
 */
export enum PortalLocation {
    MAIN = 'main',
    SIDEBAR = 'sidebar',
    SIDEBAR_CONTROL = 'sidebarControl',
    SIDEBAR_TITLE = 'sidebarTitle',
    CONSOLE = 'console',
    MAIN_TITLE = 'mainTitle',
    FOOTER_RIGHT = 'footerRight',
    FOOTER_LEFT = 'footerLeft',
}

interface IPortalProps {
    /**
     * The area where we wish to portal content to
     */
    portalLocation: PortalLocation;

    /**
     * The content to portal
     */
    children: React.ReactNode | React.ReactNodeArray;
}

/**
 * This component adds its children to a provided area of the application
 */
export const Portal = (props: IPortalProps) => {
    // Fetch the HTML node we are portaling to
    const portalNode = document.getElementById(props.portalLocation);

    if (portalNode !== null) {
        // Create a React Portal based on the node we just found
        return ReactDOM.createPortal(props.children, portalNode);
    } else {
        // Can't locate the HTML node. Something is off.
        throw new UnknownPortalException(
            `Can't locate DOM node "${
                props.portalLocation
            }". Make sure it is defined in index.html.`,
        );
    }
};
