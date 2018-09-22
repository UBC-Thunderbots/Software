import * as React from 'react';
import { Stage } from 'react-pixi-fiber';
import { Viewport } from './Viewport';

interface IVisualizerProps {
    width: number;
    height: number;
    worldWidth: number;
    worldHeight: number;
}

/**
 * The base component for a Pixi.js rendering context. Creates a canvas element and renders the components passed
 * as child. Includes a viewport that provides zoom and drag functionality.
 */
export default class Visualizer extends React.Component<IVisualizerProps> {
    public render() {
        const { width, height, worldWidth, worldHeight } = this.props;

        return (
            <Stage
                width={width}
                height={height}
                options={{
                    antialias: true,
                    backgroundColor: 0x10bb99,
                }}
            >
                <Viewport screenWidth={width} screenHeight={height} worldWidth={worldWidth} worldHeight={worldHeight}>
                    {this.props.children}
                </Viewport>
            </Stage>
        );
    }
}