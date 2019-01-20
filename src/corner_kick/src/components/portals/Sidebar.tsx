/*
 * This file defines the layout of the sidebar area of the
 * application
 */

import * as React from 'react';
import Scrollbars from 'react-custom-scrollbars';
import * as ReactDOM from 'react-dom';

/**
 * Display a sidebar with support for a scrollbar if the content
 * doesn't fit within the viewport
 */
export class Sidebar extends React.Component {
    public render() {
        /*
         * Here, we define the sidebar scrollbars and inject the children
         * content into it.
         */
        return ReactDOM.createPortal(
            <Scrollbars>{this.props.children}</Scrollbars>,
            document.getElementById('sidebar')!,
        );
    }
}
