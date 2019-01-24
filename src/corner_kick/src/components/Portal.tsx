/*
 * This file defines a React component which adds other elements
 * in the main area of the application
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';

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
    main,
    sidebar,
    sidebarControl,
    sidebarTitle,
    console,
    mainTitle,
    footerRight,
    footerLeft,
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
 * This components adds its children to the main area of the application
 */
export const Portal = (props: IPortalProps) => {
    return ReactDOM.createPortal(
        props.children,
        document.getElementById(PortalLocation[props.portalLocation])!,
    );
};
