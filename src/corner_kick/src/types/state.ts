import { Setting } from './settings';
import { IRosoutMessage } from './standardROSMessages';

export interface IRootState {
    logger: ILoggerState;
    ros: IROSState;
    settings: ISettingsState;
}

export interface ILoggerState {
    rosout: IRosoutMessage[];
}

export interface IROSState {
    status: 'connected' | 'disconnected' | 'error';
    errorMessage: string;
    nodes: string[];
    topics: string[];
    services: string[];
    params: string[];
}

export type ISettingsState = { [K in Setting]: string };
