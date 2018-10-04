import * as React from 'react';

import { Control } from '~/components/modules/Control';
import { Logger } from '~/components/modules/Logger';
import { Visualizer } from '~/components/modules/Visualizer';
import { SplitPane } from '~/components/ui/SplitPane';

/**
 * The main page of our application. 
 */
export class Main extends React.Component {

  public render() {
    return (
        <SplitPane left={<Control />} top={<Visualizer />} bottom={<Logger />} />
    );
  }
}