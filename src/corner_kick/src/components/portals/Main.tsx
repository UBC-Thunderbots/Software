/*
 * This file defines a React component which adds other elements
 * in the main area of the application
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';

/**
 * This components adds its children to the main area of the application
 */
export const Main = (props: { children: React.ReactNode }) => {
    return ReactDOM.createPortal(props.children, document.getElementById('main')!);
};
