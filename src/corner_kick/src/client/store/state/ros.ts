export interface IROSState {
    status: 'connected' | 'disconnected' | 'error';
    errorMessage: string;
    nodes: string[];
    topics: string[];
    services: string[];
    params: string[];
}
