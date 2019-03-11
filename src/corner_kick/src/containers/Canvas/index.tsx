/**
 * This file defines a Canvas element. For now, this elements report how many
 * layer messages it has received.
 */

import * as React from 'react';

import { ILayerMessage } from 'SRC/types';
import { ShapeReceiver } from './shapeReceiver';

export class Canvas extends React.Component {
    public state = {
        messageCount: 0,
    };

    private shapeReceiver: ShapeReceiver;
    private active = true;

    /**
     * Initializes the shape receiver and starts requesting messages.
     */
    public componentDidMount() {
        this.shapeReceiver = new ShapeReceiver(this.handleLayer);
        this.shapeReceiver.connect();

        setTimeout(this.requestConnection, 16);
    }

    /**
     * Display the number of messages received.
     */
    public render() {
        return (
            <div>
                Received messages: {this.state.messageCount}. Check the browser console
                for more info.
            </div>
        );
    }

    /**
     * Clean-up after ourselves to avoid memory leaks
     */
    public componentWillUnmount() {
        this.active = false;
        this.shapeReceiver.close();
    }

    /**
     * Here, we request data from the ROS shape websocket.
     */
    private requestConnection = () => {
        this.shapeReceiver.requestData();

        if (this.active === true) {
            setTimeout(this.requestConnection, 16);
        }
    };

    /**
     * Update message count and display message in the browser console when we receive a
     * new message
     */
    private handleLayer = (layer: ILayerMessage) => {
        this.setState({ messageCount: this.state.messageCount + 1 });
        console.log(layer);
    };
}
