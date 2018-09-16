import * as React from 'react';
import {Provider} from 'unstated';

import { Header } from './containers/Header';
import { ROSService } from './services/ros';

class App extends React.Component {

  public render() {
    return (
      <Provider>
        <ROSService />
        <Header />
      </Provider>
    );
  }
}

export default App;
