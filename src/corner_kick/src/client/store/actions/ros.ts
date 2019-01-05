import { createAction } from 'typesafe-actions';

export const start = createAction('ros/START');
export const stop = createAction('ros/STOP');

export const connected = createAction('ros/CONNECTED');
export const disconnected = createAction('ros/DISCONNECTED');
export const error = createAction('ros/ERROR', (resolve) => {
    return (errorMessage: string) => resolve({ errorMessage });
});

export const subscribeTopic = createAction('ros/SUBSCRIBE_TOPIC', (resolve) => {
    return (topic: string, messageType: string) => resolve({ topic, messageType });
});
export const unsubscribeTopic = createAction('ros/UNSUBSCRIBE_TOPIC', (resolve) => {
    return (topic: string, messageType: string) => resolve({ topic, messageType });
});
export const newMessage = createAction('ros/NEW_MESSAGE', (resolve) => {
    return (topic: string, message: any) => resolve({ topic, message });
});

export const setNodes = createAction('ros/SET_NODES', (resolve) => {
    return (nodes: string[]) => resolve({ nodes });
});
export const setTopics = createAction('ros/SET_TOPICS', (resolve) => {
    return (topics: string[]) => resolve({ topics });
});
export const setServices = createAction('ros/SET_SERVICES', (resolve) => {
    return (services: string[]) => resolve({ services });
});
export const setParams = createAction('ros/SET_PARAMS', (resolve) => {
    return (params: string[]) => resolve({ params });
});
