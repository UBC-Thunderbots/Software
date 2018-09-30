import * as React from 'react';
import {Provider} from 'unstated';

import { Main } from './pages/Main';

class App extends React.Component {

  public render() {
    return (
      <Provider>
          <Main />
      </Provider>
    );
  }
}

export default App;
