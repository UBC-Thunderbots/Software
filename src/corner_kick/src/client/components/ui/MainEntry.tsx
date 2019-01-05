import { dark } from 'ayu';
import * as React from 'react';
import styled from 'styled-components';

const Wrapper = styled.div`
    position: relative;

    width: 192px;
    height: 40px;
    min-width: 128px;
    padding: 8px;

    display: flex;
    align-items: center;

    border-right: 1px solid ${dark.ui.line.hex()};

    cursor: pointer;

    color: ${dark.syntax.comment.hex()};

    transition: all 0.2s;

    &:hover .clear {
        display: initial;
    }

    &.active {
        color: ${dark.common.fg.hex()};
        background: ${dark.ui.panel.bg.hex()};
        cursor: initial;
    }

    &.active:after {
        position: absolute;
        left: 0;
        right: 0;
        top: 0;

        height: 2px;

        background: ${dark.syntax.string.hex()};

        content: '';
    }

    &.active .clear {
        display: initial;
    }
`;

const Icon = styled.i`
    display: none;
    padding: 4px;
    transition: all 0.2s;
    border-radius: 4px;
    margin-left: auto;
    font-size: 16px;
    cursor: pointer;

    &:hover {
        background: ${dark.ui.selection.bg};
    }
`;

interface IMainEntry {
    text: string;
    active: boolean;
    onClick?: (e: React.MouseEvent) => void;
}

export class MainEntry extends React.Component<IMainEntry> {
    public render() {
        return (
            <Wrapper
                className={this.props.active ? 'active' : ''}
                onClick={this.props.onClick}
            >
                {this.props.text}
                <Icon className="material-icons clear">clear</Icon>
            </Wrapper>
        );
    }
}
