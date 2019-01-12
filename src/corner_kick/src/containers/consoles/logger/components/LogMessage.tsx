/**
 * Defines a React component displaying a single logger message
 */

import * as React from 'react';

import styled from 'SRC/utils/styled-components';

const DEBUG = 1;
const INFO = 2;
const WARN = 4;
const ERROR = 8;
const FATAL = 16;

// Here, we assign a specific color for each severity level
const Wrapper = styled.div`
    width: 100%;

    &.debug {
        color: ${(props) => props.theme.colors.debug};
    }

    &.info {
        color: ${(props) => props.theme.colors.info};
    }

    &.warn {
        color: ${(props) => props.theme.colors.warn};
    }

    &.error {
        color: ${(props) => props.theme.colors.error};
    }
`;

interface ILogMessageProps {
    name: string;
    msg: string;
    level: number;
}

/**
 * Displays a single log message from /rosout ROS topic. Ensures the correct
 * color is used for each severity level
 */
export const LogMessage = (props: ILogMessageProps) => {
    let severity = '';

    switch (props.level) {
        case DEBUG:
            severity = 'debug';
            break;
        case INFO:
            severity = 'info';
            break;
        case WARN:
            severity = 'warn';
            break;
        case ERROR:
        case FATAL:
            severity = 'error';
    }

    return (
        <Wrapper className={severity}>
            <b>
                {severity} [{props.name}]:{' '}
            </b>
            {props.msg}
        </Wrapper>
    );
};
