import * as React from 'react';
import {Provider} from 'unstated';

import { Main } from './pages/Main';

/**
 * Primary React component. This is where our top-level application is defined.
 */
class App extends React.Component {

  // TODO: add support for navigating between pages. For now, we only have Main to visit.
  public render() {
    return (
      <Provider>
          <Main />
      </Provider>
    );
  }
}

export default App;
