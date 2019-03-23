/*
 * This file specifies the mount point for our application
 */

import * as React from 'react';
import * as ReactDom from 'react-dom';

import { App } from './App';

import doc from 'THUNDERBOTS_ROOT/src/thunderbots/software/util/parameter/config/ai.yaml';

console.log(doc);

// Mount the React component App to the div `#react`
ReactDom.render(React.createElement(App), document.getElementById('react'));
