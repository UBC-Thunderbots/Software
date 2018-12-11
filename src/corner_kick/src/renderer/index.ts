/**
 * @fileoverview Entry point of our application.
 */
import * as React from 'react';
import * as ReactDOM from 'react-dom';

import App from 'RENDERER/App';

// Base styling
import './index.css';

// Our "main" method. This is where our React app begins, by adding itself to the root div
// element.
ReactDOM.render(React.createElement(App), document.getElementById('root') as HTMLElement);
