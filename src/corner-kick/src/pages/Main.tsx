import * as React from 'react';

import SplitPane from 'react-split-pane';
import { Logger } from '../modules/logger';

export class Main extends React.Component {

  public render() {
    return (
        <SplitPane split="vertical" minSize={50}>
            <div />
            <SplitPane split="horizontal">
                <div />
                <Logger />
            </SplitPane>
        </SplitPane> 
    );
  }
}