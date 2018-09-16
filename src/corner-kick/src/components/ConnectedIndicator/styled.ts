import styled, {keyframes} from 'styled-components';

export const StyledWrapper = styled.div`
    width: 32px;
    height: 64px;
    display: flex;
    justify-content: center;
    align-items: center;
    transition: all .2s;
    cursor: pointer;

    &:hover {
        background: #EEE;
    }
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

