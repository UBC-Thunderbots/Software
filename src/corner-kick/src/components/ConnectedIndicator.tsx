import * as React from 'react';
import styled, {keyframes} from 'styled-components';

import { ROSState } from '../services/ros';

export const StyledWrapper = styled.div`
    display: flex;
    justify-content: center;
    align-items: center;
    transition: all .2s;
`;


const StyledPulse = keyframes`
    0% {
        opacity: 0;
    }

    50% {
        opacity: 1;
    }

    100% {
        opacity: 0;
    }
`

export const StyledDot = styled.div`
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #AAA;
    transition: all .2s;

    &.connected {
        background: #4FAC50;
    }

    &.disconnected {
        background: #AAA;
    }

    &.connecting {
        animation: ${StyledPulse} 1s ease-in-out infinite;
    }

    &.error {
        background: #F34439;
    }
`;

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