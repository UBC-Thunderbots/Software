import * as React from 'react';
import * as ReactDOM from 'react-dom';

import App from './App';

import "@blueprintjs/core/lib/css/blueprint.css";
import "@blueprintjs/icons/lib/css/blueprint-icons.css";
import './index.css';

ReactDOM.render(
  React.createElement(App),
  document.getElementById('root') as HTMLElement
);
