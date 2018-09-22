import * as React from 'react';
import {Provider} from 'unstated';

import Visualizer from './components/Visualizer';
import { Wrapper } from './components/Wrapper';

import { Header } from './containers/Header';

import { ROSService } from './services/ros';

class App extends React.Component {

  public render() {
    return (
      <Provider>
        <ROSService />
        <Wrapper>
          <Header />
          <Visualizer width={100} height={100} worldHeight={100} worldWidth={100} />
        </Wrapper>
      </Provider>
    );
  }
}

export default App;
