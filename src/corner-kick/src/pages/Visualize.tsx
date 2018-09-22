import * as React from 'react';
import { Subscribe } from 'unstated';

import { Circle } from '../components/Circle';
import Visualizer from '../components/Visualizer';

import { ROSContainer } from '../services/ros';

class Visualize extends React.Component {

  public render() {
    return (
        <Subscribe to={[ROSContainer]}>
            { (ros: ROSContainer) => {
                return (
                    <Visualizer width={800} height={800} worldHeight={100} worldWidth={100}>
                        <Circle
                            name='center-circle'
                            x={ros.state.topic['/turtle1/pose'] ? ros.state.topic['/turtle1/pose'].x * 50 : 0}
                            y={ros.state.topic['/turtle1/pose'] ? -ros.state.topic['/turtle1/pose'].y * 50 : 0}
                            radius={6}
                            fill={0xFF0000}
                        />
                    </Visualizer>
                );
            }}
        </Subscribe>
    );
  }
}

export {Visualize};
