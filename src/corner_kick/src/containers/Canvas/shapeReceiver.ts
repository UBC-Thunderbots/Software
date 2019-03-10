/**
 * This file defines the logic that connects to the ROS shape websocket and
 * processes new layer data.
 */

import { ILayerMessage } from 'SRC/types/canvas';
import { parseLayer } from './utils/parseLayer';

/**
 * Type of the callback called when new layer data is received
 */
type ShapeReceiverCallback = (layer: ILayerMessage) => void;

/**
 * Connects and manages the connection between the visualizer and
 * the layer data websocket.
 */
export class ShapeReceiver {
    private requestBuffer = new ArrayBuffer(1);
    private ws: WebSocket;

    private callback: ShapeReceiverCallback;

    constructor(callback: ShapeReceiverCallback) {
        this.callback = callback;

        const dataView = new DataView(this.requestBuffer);
        dataView.setUint8(0, 1);
    }

    /**
     * Connects to the websocket
     */
    public connect = () => {
        this.ws = new WebSocket('ws://localhost:9091');
        this.ws.binaryType = 'arraybuffer';
        this.ws.addEventListener('message', (event: MessageEvent) =>
            this.handleData(event.data),
        );
    };

    /**
     * Request new layer from the websocket. This function should be called when
     * the browser is idling.
     */
    public requestData = () => {
        this.ws.send(this.requestBuffer);
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
        this.callback(parseLayer(data));
    };
}
