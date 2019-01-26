/*
 * This file defines a React component which adds other elements
 * in the console area of the application
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';

/**
 * This components adds its children to the console area of the application
 */
export const Console = (props: { children: React.ReactNode }) => {
    return ReactDOM.createPortal(props.children, document.getElementById('console')!);
};
