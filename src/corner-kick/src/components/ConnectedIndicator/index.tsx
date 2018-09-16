import * as React from 'react';

import { ROSState } from '../../services/ros';

import {
    StyledDot,
    StyledWrapper,
} from './styled';

export const ConnectedIndicator = (props: {state: ROSState}) => {
    let state = '';
    switch(props.state) {
        case ROSState.Connected:
        state = 'connected';
        break;
        case ROSState.Disconnected:
        state = 'disconnected';
        break;
        case ROSState.Connecting:
        state = 'connecting';
        break;
        case ROSState.Error:
        state = 'error';
        break;
    }
    return (
        <StyledWrapper>
            <StyledDot className={state}/>
        </StyledWrapper>
    )
}