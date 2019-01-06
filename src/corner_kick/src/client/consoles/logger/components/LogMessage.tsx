import { dark } from 'ayu';
import * as React from 'react';
import styled from 'styled-components';

const DEBUG = 1;
const INFO = 2;
const WARN = 4;
const ERROR = 8;
const FATAL = 16;

const Wrapper = styled.div`
    width: 100%;
`;

interface ILogMessageProps {
    name: string;
    msg: string;
    level: number;
}

export const LogMessage = (props: ILogMessageProps) => {
    let severity = '';
    let color = dark.common.ui.hex();

    switch (props.level) {
        case DEBUG:
            severity = 'debug';
            color = dark.common.ui.hex();
            break;
        case INFO:
            severity = 'info';
            color = dark.syntax.tag.hex();
            break;
        case WARN:
            severity = 'WARN';
            color = dark.syntax.func.hex();
            break;
        case ERROR:
        case FATAL:
            severity = 'ERROR';
            color = dark.syntax.error.hex();
    }

    return (
        <Wrapper style={{ color }}>
            <b>
                {severity} [{props.name}]:{' '}
            </b>
            {props.msg}
        </Wrapper>
    );
};
