import * as React from 'react';
import {Provider} from 'unstated';

import { Wrapper } from './components/Wrapper';

import { Header } from './containers/Header';

import { ROSService } from './services/ros';

import { Visualize } from './pages/Visualize';

class App extends React.Component {

  public render() {
    return (
      <Provider>
        <ROSService />
        <Wrapper>
          <Header />
          <Visualize />
        </Wrapper>
      </Provider>
    );
  }
}

export default App;
