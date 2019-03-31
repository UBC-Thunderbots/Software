const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 9091 });

let j = 0;
const array1 = new ArrayBuffer(500 * 15 + 1);
const dataView1 = new DataView(array1);
const array2 = new ArrayBuffer(500 * 15 + 1);
const dataView2 = new DataView(array2);
const array3 = new ArrayBuffer(1 * 15 + 1);
const dataView3 = new DataView(array3);

dataView1.setUint8(0, 0);
dataView2.setUint8(0, 1);
dataView3.setUint8(0, 2);

const render = () => {
    for (let i = 0; i < 500; ++i) {
        dataView1.setUint8(15 * i + 1, j % 2);
        dataView1.setInt16(15 * i + 2, (i % 20) * 240);
        dataView1.setInt16(15 * i + 4, Math.floor(i / 20) * 120);
        dataView1.setInt16(15 * i + 6, 120);
        dataView1.setInt16(15 * i + 8, 120);
        dataView1.setInt16(15 * i + 10, j % 360);
        dataView1.setUint8(15 * i + 12, 255);
        dataView1.setUint8(15 * i + 13, 255);
        dataView1.setUint8(15 * i + 14, 0);
        dataView1.setUint8(15 * i + 15, 0);

        j++;
    }
    for (let i = 0; i < 500; ++i) {
        dataView2.setUint8(15 * i + 1, j % 2);
        dataView2.setInt16(15 * i + 2, (i % 20) * 240 + 120);
        dataView2.setInt16(15 * i + 4, Math.floor(i / 20) * 120);
        dataView2.setInt16(15 * i + 6, 120);
        dataView2.setInt16(15 * i + 8, 120);
        dataView2.setInt16(15 * i + 10, j % 360);
        dataView2.setUint8(15 * i + 12, 255);
        dataView2.setUint8(15 * i + 13, 0);
        dataView2.setUint8(15 * i + 14, 255);
        dataView2.setUint8(15 * i + 15, 0);

        j++;
    }
    dataView3.setUint8(1, 0);
    dataView3.setInt16(2, 0);
    dataView3.setInt16(4, 0);
    dataView3.setInt16(6, 120 * 40);
    dataView3.setInt16(8, 120 * 25);
    dataView3.setInt16(10, Math.round(j / 10000) % 360);
    dataView3.setUint8(12, 255);
    dataView3.setUint8(13, 0);
    dataView3.setUint8(14, 0);
    dataView3.setUint8(15, 255);

    setTimeout(render, 16);
};

const send = (ws) => {
    ws.send(array3);
    ws.send(array1);
    ws.send(array2);
    setTimeout(send, 16, ws);
};

wss.on('connection', function open(ws) {
    setTimeout(send, 32, ws);
});

render();
