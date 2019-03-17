/**
 * This file is for testing layer data parsing
 */

import { Server } from 'mock-socket';

import { ILayerMessage } from 'SRC/types';

import { LAYER_WEBSOCKET_ADDRESS } from 'SRC/constants';
import { generateLayerBuffer, generateRandomShapes } from '../__helpers__/shapes';
import { LayerReceiver } from '../layerReceiver';

describe('parseLayer', () => {
    describe('when we process layer data from the websocket', () => {
        const layer = 2;
        const flags = 1;
        const shapeCount = 100;
        const shapes = generateRandomShapes(shapeCount);

        let arrayBuffer: ArrayBuffer;

        beforeAll(() => {
            arrayBuffer = generateLayerBuffer(layer, flags, shapes);
        });

        it('correctly processes the message', (done) => {
            const fakeURL = LAYER_WEBSOCKET_ADDRESS;
            const mockServer = new Server(fakeURL);

            // We create a fake websocket server and send a single layer message
            // on connection
            mockServer.on('connection', (socket) => {
                socket.binaryType = 'arraybuffer';
                socket.send(arrayBuffer);
            });

            // The callback calls done after running assertions. This
            // is to notify jest that the test is complete
            const layerCallback = (layerData: ILayerMessage) => {
                expect(layerData.layer).toEqual(layer);
                expect(layerData.shapes.length).toEqual(shapeCount);
                expect(layerData.shapes).toStrictEqual(shapes);
                layerReceiver.close();
                done();
            };

            // We create a layer receiver and tell it to connect to
            // the server
            const layerReceiver = new LayerReceiver(layerCallback);
            layerReceiver.connect();
        });
    });
});
