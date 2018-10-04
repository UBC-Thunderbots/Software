import * as React from 'react';
import * as ReactDOM from 'react-dom';

import App from './App';

// Base styling
import "@blueprintjs/core/lib/css/blueprint.css";
import "@blueprintjs/icons/lib/css/blueprint-icons.css";
import './index.css';

// Our "main" method. This is where our React app starts, by adding itself to the root div
// element.
ReactDOM.render(
  React.createElement(App),
  document.getElementById('root') as HTMLElement
);
