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
 * -   **console:** Contains views visible across the application
 */
export enum PortalLocation {
    MAIN = 'main',
    SIDEBAR = 'sidebar',
    SIDEBAR_TITLE = 'sidebarTitle',
    CONSOLE = 'console',
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
