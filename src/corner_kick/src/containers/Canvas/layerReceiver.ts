/**
 * This file defines the logic that connects to the ROS layer websocket and
 * processes new layer data.
 */

import { LAYER_WEBSOCKET_ADDRESS } from 'SRC/constants';

/**
 * Type of the callback called when new layer data is received
 */
type ShapeReceiverCallback = (data: ArrayBuffer) => void;

/**
 * Connects and manages the connection between the visualizer and
 * the layer data websocket.
 */
export class LayerReceiver {
    private ws: WebSocket;

    private callback: ShapeReceiverCallback;

    constructor(callback: ShapeReceiverCallback) {
        this.callback = callback;
    }

    /**
     * Connects to the websocket
     */
    public connect = (url: string = LAYER_WEBSOCKET_ADDRESS) => {
        this.ws = new WebSocket(url);
        this.ws.binaryType = 'arraybuffer';
        this.ws.addEventListener('message', (event: MessageEvent) =>
            this.handleData(event.data),
        );
    };

    /**
     * Close the websocket connection. MUST be called
     * to avoid memory leaks.
     */
    public close = () => {
        this.ws.close();
    };

    /**
     * Parse data from the websocket
     */
    private handleData = (data: ArrayBuffer) => {
        this.callback(data);
    };
}
