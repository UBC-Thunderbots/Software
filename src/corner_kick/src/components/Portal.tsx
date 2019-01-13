/**
 * This file defines a React component which adds other elements
 * in the main area of the application
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';

interface IPortalProps {
    to:
        | 'main'
        | 'sidebar'
        | 'sidebarControl'
        | 'sidebarTitle'
        | 'console'
        | 'mainTitle'
        | 'footerRight'
        | 'footerLeft';
    children: React.ReactNode | React.ReactNodeArray;
}

/**
 * This components adds its children to the main area of the application
 */
export const Portal = (props: IPortalProps) => {
    return ReactDOM.createPortal(props.children, document.getElementById(props.to)!);
};
