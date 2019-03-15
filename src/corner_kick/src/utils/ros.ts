import { Message, Ros, ServiceRequest, Topic } from 'roslib';

const ros: Ros = new Ros({});
let isConnected = false;

export const connect = (url: string = 'ws://localhost:9090') => {
    ros.connect(url);
    ros.on('connection', () => (isConnected = true));
    ros.on('error', () => (isConnected = false));
    ros.on('close', () => (isConnected = false)); // TODO Add reconnectiobn logic
};

export const subscribeToROSTopic = (
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
) => {
    if (isConnected) {
        const topic = new Topic({
            messageType,
            name,
            ros,
        });
        topic.subscribe(callback);
    }
};

export const unsubscribeToROSTopic = (
    name: string,
    messageType: string,
    callback: (message: Message) => void,
) => {
    if (isConnected) {
        const topic = new Topic({
            messageType,
            name,
            ros,
        });
        topic.unsubscribe(callback);
    }
};

export const getParams = (timeout: number = 100) => {
    return new Promise<string[]>((resolve, reject) => {
        ros.getParams((params) => {
            resolve(params);
        });

        // We reject if the timeout has elapsed
        setTimeout(() => reject(), timeout);
    });
};

export const getServices = (timeout: number = 100) => {
    return new Promise<string[]>((resolve, reject) => {
        ros.getServices((params) => {
            resolve(params);
        });

        // We reject if the timeout has elapsed
        setTimeout(() => reject(), timeout);
    });
};

export const getServiceRequests = (timeout: number = 100) => {
    return new Promise<string[]>
}
