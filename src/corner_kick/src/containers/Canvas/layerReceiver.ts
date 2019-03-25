/**
 * This file defines the logic that connects to the ROS layer websocket and
 * processes new layer data.
 */

import { BYTES_PER_SHAPE, LAYER_WEBSOCKET_ADDRESS } from 'SRC/constants';
import { ILayerMessage, ISprite } from 'SRC/types';
import { MalformedShapesException } from 'SRC/utils/exceptions';

/**
 * Type of the callback called when new layer data is received
 */
type ShapeReceiverCallback = (layer: ILayerMessage) => void;

/**
 * Parses an ArrayBuffer into a LayerMessage.
 * @throws MalformedShapeException if the ArrayBuffer contains an illegal byte count
 */
const parseLayer = (data: ArrayBuffer): ILayerMessage => {
    const incomingDataView = new DataView(data);

    // Parse the layer number
    const layer = incomingDataView.getUint8(0);

    // We expect a certain multiple of bytes, based on sprite size
    const incomingSpriteCount = (data.byteLength - 1) / BYTES_PER_SHAPE;
    if (incomingSpriteCount !== Math.round(incomingSpriteCount)) {
        throw new MalformedShapesException(
            'The message from the server contains malformed data',
        );
    } else {
        // Parse each sprites
        const sprites = new Array(incomingSpriteCount).fill(true).map(
            (_, index): ISprite => {
                // We start at 1 as the first byte of the message
                // contain the layer number
                const startPos = 1 + index * BYTES_PER_SHAPE;
                return {
                    texture: incomingDataView.getUint8(startPos),
                    x: incomingDataView.getInt16(startPos + 1),
                    y: incomingDataView.getInt16(startPos + 3),
                    width: incomingDataView.getInt16(startPos + 5),
                    height: incomingDataView.getInt16(startPos + 7),
                    rotation: incomingDataView.getInt16(startPos + 9),
                    opacity: incomingDataView.getUint8(startPos + 11),
                    red: incomingDataView.getUint8(startPos + 12),
                    green: incomingDataView.getUint8(startPos + 13),
                    blue: incomingDataView.getUint8(startPos + 14),
                };
            },
        );
        return {
            layer,
            sprites,
        };
    }
};

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
        const parsedLayer = parseLayer(data);

        this.callback(parsedLayer);
    };
}
