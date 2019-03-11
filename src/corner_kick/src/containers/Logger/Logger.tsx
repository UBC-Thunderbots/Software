/***
 * This file specifies a Logger outputting info from /rosout.
 */

import * as React from 'react';
import { FixedSizeList as List } from 'react-window';

import { IRosoutMessage } from 'SRC/types';

import { CustomScrollbars } from './CustomScrollbar';
import { LogItem } from './LogItem';

interface ILoggerProps {
    messages: IRosoutMessage[];
}

/**
 * Logger view, displaying messages received from /rosout.
 */
export class Logger extends React.Component<ILoggerProps> {
    /**
     * Displays the correct /rosout item based on index position.
     */
    public Row = ({ index, style }: { index: number; style: {} }) => (
        <LogItem item={this.props.messages[index]} style={style} />
    );

    public render() {
        return (
            <List
                height={300}
                itemCount={this.props.messages.length}
                itemSize={20}
                width="100%"
                outerElementType={CustomScrollbars}
            >
                {this.Row}
            </List>
        );
    }
}
