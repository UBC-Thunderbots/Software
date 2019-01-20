/*
 * This file specifies the mount point for our application
 */

import * as React from 'react';
import * as ReactDom from 'react-dom';

import './style/base.css';

import { App } from './App';

// Mount the React component App to the div `#react`
ReactDom.render(React.createElement(App), document.getElementById('react'));
