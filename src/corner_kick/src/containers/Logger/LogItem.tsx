/**
 * This file specifies the implementation of a logger item
 */

import * as React from 'react';

import { IRosoutMessage, RosoutLevel } from 'SRC/types';
import styled from 'SRC/utils/styled-components';

/**
 * Specifies most of the styling for the LogItem
 * It also includes level dependent styling.
 */
const Wrapper = styled('div')<{ level: number }>`
    width: 100%;

    padding: 0 16px;

    display: flex;
    justify-content: space-between;
    align-items: center;

    border-bottom: 1px solid ${(props) => props.theme.colors.border};

    font-size: 12px;

    ${(props) => {
        switch (props.level) {
            case RosoutLevel.DEBUG:
                return `color: ${props.theme.colors.subdued};`;
            case RosoutLevel.INFO:
                return `color: ${props.theme.colors.fg};`;
            case RosoutLevel.WARN:
                return `color: ${props.theme.colors.warn};`;
            case RosoutLevel.ERROR:
                return `color: ${props.theme.colors.error};`;
            case RosoutLevel.FATAL:
                return `
                    color: ${props.theme.colors.error};
                    font-weight: 900;
                `;
            default:
                return `color: ${props.theme.colors.fg};`;
        }
    }}

    &:last-child {
        border-bottom: none;
    }
`;

/**
 * Styling for the body section (name and msg) of a log message. Ensures that
 * the message remains one line and adds ellipses if this is not the case.
 */
const Body = styled.span`
    height: 100%;

    padding-right: 8px;

    user-select: text;

    flex-grow: 1;
    overflow: hidden;
    white-space: nowrap;
    text-overflow: ellipsis;
`;

/**
 * Styling for the file section (filename and line number) of a log message.
 */
const File = styled.span`
    text-decoration: underline;
`;

export const LogItem = ({ item, style }: { item: IRosoutMessage; style: {} }) => {
    // We shorten the path to only include the filename
    const shortFilename = item.file.match(/[^/]*$/)![0];

    // Message to be displayed when the user hovers over a log item.
    const hoverText =
        `name: ${item.name}\n` +
        `level: ${RosoutLevel[item.level]}\n` +
        `msg: ${item.msg}\n` +
        `file: ${item.file}\n` +
        `line: ${item.line}`;

    return (
        <Wrapper level={item.level} style={style} title={hoverText}>
            <Body>
                {item.name}: {item.msg}
            </Body>
            <File>
                {shortFilename}:{item.line}
            </File>
        </Wrapper>
    );
};
