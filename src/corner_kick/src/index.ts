/*
 * This file specifies the mount point for our application
 */

import * as React from 'react';
import * as ReactDom from 'react-dom';

import { App } from './App';

import '@blueprintjs/core/lib/css/blueprint.css';
import '@blueprintjs/icons/lib/css/blueprint-icons.css';

// Mount the React component App to the div `#react`
ReactDom.render(React.createElement(App), document.getElementById('react'));
