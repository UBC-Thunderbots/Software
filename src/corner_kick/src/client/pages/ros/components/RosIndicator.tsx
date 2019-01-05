import { dark } from 'ayu';
import * as React from 'react';
import styled from 'styled-components';

const Wrapper = styled.div`
    width: 100%;
    height: 128px;

    border-bottom: 1px solid ${dark.ui.line};

    display: flex;
    flex-flow: column nowrap;
    justify-content: center;
    align-items: center;
`;

interface IRosIndicator {
    status: string;
    errorMessage: string;
}

export const RosIndicator = (props: IRosIndicator) => {
    switch (props.status) {
        case 'connected':
            return (
                <Wrapper style={{ color: dark.syntax.string.hex() }}>
                    <i
                        className="material-icons"
                        style={{
                            fontSize: '32px',
                        }}
                    >
                        check_circle_outline
                    </i>
                </Wrapper>
            );
        case 'disconnected':
            return (
                <Wrapper style={{ color: dark.syntax.comment.hex() }}>
                    <i
                        className="material-icons"
                        style={{
                            fontSize: '32px',
                        }}
                    >
                        highlight_off
                    </i>
                </Wrapper>
            );
        case 'error':
            return (
                <Wrapper style={{ color: dark.syntax.error.hex() }}>
                    <i
                        className="material-icons"
                        style={{
                            fontSize: '32px',
                        }}
                    >
                        error_outline
                    </i>
                    <p style={{ color: dark.common.fg.hex() }}>{props.errorMessage}</p>
                </Wrapper>
            );
        default:
            return null;
    }
};
