const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 9091 });

let j = 0;
const array = new ArrayBuffer(1000 * 15 + 1);
const dataView = new DataView(array);

dataView.setUint8(0, 0);

const render = () => {
    for (let i = 0; i < 1000; ++i) {
        dataView.setUint8(15 * i + 1, j % 2);
        dataView.setInt16(15 * i + 2, (i % 100) * 120);
        dataView.setInt16(15 * i + 4, Math.floor(i / 100) * 120);
        dataView.setInt16(15 * i + 6, 120);
        dataView.setInt16(15 * i + 8, 120);
        dataView.setInt16(15 * i + 10, j % 360);
        dataView.setUint8(15 * i + 12, 255);
        dataView.setUint8(15 * i + 13, 255);
        dataView.setUint8(15 * i + 14, 255);
        dataView.setUint8(15 * i + 15, 255);

        j++;
    }
    setTimeout(render, 16);
};

const send = (ws) => {
    ws.send(array);
    setTimeout(send, 32, ws);
};

wss.on('connection', function open(ws) {
    setTimeout(send, 32, ws);
});

render();
