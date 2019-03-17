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

            mockServer.on('connection', (socket) => {
                socket.binaryType = 'arraybuffer';
                socket.send(arrayBuffer);
            });

            const layerCallback = (layerData: ILayerMessage) => {
                expect(layerData.layer).toEqual(layer);
                expect(layerData.shapes.length).toEqual(shapeCount);
                expect(layerData.shapes).toStrictEqual(shapes);
                layerReceiver.close();
                done();
            };

            const layerReceiver = new LayerReceiver(layerCallback);
            layerReceiver.connect();
        });
    });
});
