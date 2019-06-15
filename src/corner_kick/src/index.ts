/*
 * This file specifies the mount point for our application
 */

import { FocusStyleManager } from '@blueprintjs/core';
import * as React from 'react';
import * as ReactDom from 'react-dom';

import { App } from './App';

import '@blueprintjs/core/lib/css/blueprint.css';
import '@blueprintjs/icons/lib/css/blueprint-icons.css';

FocusStyleManager.onlyShowFocusOnTabs();

// Mount the React component App to the div `#react`
ReactDom.render(React.createElement(App), document.getElementById('react'));
