import * as React from 'react';
import ROSLib from 'roslib';
import {Container, Subscribe} from 'unstated';

export class ROSService extends React.Component {
    public render() {
        return (
            <Subscribe to={[ROSContainer]}>
                {(ros: ROSContainer) => {
                    ros.connect();
                    return null;
                }}
            </Subscribe>
        )
    }
}

interface IRosState {
    state: ROSState;
    error: string;
}

export enum ROSState {
    Connected,
    Connecting,
    Disconnected,
    Error
}

export class ROSContainer extends Container<IRosState> {
    public state = {
        error: '',
        state: ROSState.Disconnected,
    };

    private ros: any;

    public connect = () => {
        if(this.state.state !== ROSState.Disconnected) {
            return;
        }

        this.connectToROS('ws:localhost:9090');
    }

    private connectToROS = async (url: string) => {
        this.ros = new ROSLib.Ros({
            url
        })

        await this.setState({state: ROSState.Connecting});

        this.ros.on('connection', () => {
            this.setState({state: ROSState.Connected});
        });

        this.ros.on('error', (error: string) => {
            this.setState({state: ROSState.Error, error});
        })

        this.ros.on('close', () => {
            this.setState({state: ROSState.Disconnected});
        });
    }
}