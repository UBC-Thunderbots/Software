/**
 * This file defines the logic that receives ROS shape messages and passes them
 * on to be rendered in the Canvas.
 */

import { decode } from 'base64-arraybuffer';

import { ILayerMessage } from 'SRC/types';
import * as ROS from 'SRC/utils/ros';

/**
 * Type of the callback called when new layer data is received
 */
type ShapeReceiverCallback = (data: ArrayBuffer) => void;

/**
 * Receives and parses layer messages received from ROS.
 */
export class LayerReceiver {
    private callback: ShapeReceiverCallback;

    /**
     * Creates a new LayerReceiver
     * @param callback the callback to call when we receive new layer data
     */
    constructor(callback: ShapeReceiverCallback) {
        this.callback = callback;
    }

    /**
     * Connects to the ROS layer topic
     */
    public connect = () => {
        ROS.subscribeToROSTopic(
            '/visualizer/layers',
            'thunderbots_msgs/CanvasLayer',
            this.handleROSMessage,
            16,
        );
    };

    /**
     * Unsubscribes from the ROS layer topic
     */
    public close = () => {
        ROS.unsubscribeToROSTopic(
            '/visualizer/layers',
            'thunderbots_msgs/CanvasLayer',
            this.handleROSMessage,
        );
    };

    /**
     * We received the layer data (containing all sprite information)
     * in the form of a base64 string. We convert it to an ArrayBuffer for
     * further processing.
     */
    private handleROSMessage = (message: ILayerMessage) => {
        this.callback(decode(message.data));
    };
}
