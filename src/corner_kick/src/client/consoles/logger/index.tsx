import * as React from 'react';
import { connect } from 'react-redux';
import styled from 'styled-components';

import { IRootState } from 'SRC/store';
import { IRosoutMessage } from 'SRC/store/state/logger';

import { LogMessage } from './components/LogMessage';

const Wrapper = styled.div`
    width: 100%;
    height: 100%;

    padding: 16px;
`;

const mapStateToProps = (state: IRootState) => ({
    rosout: state.logger.rosout,
});

interface ILoggerConsoleProps {
    rosout: IRosoutMessage[];
}

class LoggerConsoleInternal extends React.Component<ILoggerConsoleProps> {
    public render() {
        return (
            <Wrapper>
                {this.props.rosout.map((message) => (
                    <LogMessage {...message} />
                ))}
            </Wrapper>
        );
    }
}

export const LoggerConsole = connect(mapStateToProps)(LoggerConsoleInternal);
