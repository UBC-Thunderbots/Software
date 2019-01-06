export interface ILoggerState {
    rosout: IRosoutMessage[];
}

export interface IRosoutMessage {
    level: number;
    msg: string;
    name: string;
}
