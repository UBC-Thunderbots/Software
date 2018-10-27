import * as React from 'react';
import { Provider } from 'unstated';

import { Logger } from '~/modules/Logger';
import { Visualizer } from '~/modules/Visualizer';
import { SplitPane } from '~/components/ui/SplitPane';

/**
 * The main page of our application. 
 */
export default class App extends React.Component {

  public render() {
    return (
      <Provider>
        <SplitPane top={<Visualizer />} bottom={<Logger />} />
      </Provider>
    );
  }
}