import * as _ from 'lodash';
import * as React from 'react';
import { Stage } from "react-pixi-fiber";

import { IResizeEntry, ResizeSensor } from "@blueprintjs/core";

/**
 * A React element to display a canvas.
 */
export class Canvas extends React.Component {

    public state = {
        height: 0,
        width: 0,
    };

    public render() {
        return (
            <ResizeSensor observeParents={true} onResize={_.debounce(this.onResize, 16)}>
                <Stage width={this.state.width} height={this.state.height} options={{
                    antialias: true,
                }}>
                    {this.props.children}
                </Stage>
            </ResizeSensor>
        );
    }

    public onResize = (entries: IResizeEntry[]) => {
        if(entries[1] !== undefined) {
            const {width, height} = entries[1].contentRect;
            this.setState({width, height});
        }
    }
}