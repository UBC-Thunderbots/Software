import * as React from 'react';

import styled from 'SRC/utils/styled-components';

const Wrapper = styled.div`
    width: 100%;
    height: 128px;

    border-bottom: 1px solid ${(props) => props.theme.colors.border};

    display: flex;
    flex-flow: column nowrap;
    justify-content: center;
    align-items: center;

    &.connected {
        color: ${(props) => props.theme.colors.success};
    }

    &.disconnected {
        color: ${(props) => props.theme.colors.subdued};
    }

    &.error {
        color: ${(props) => props.theme.colors.error};
    }
`;

interface IRosIndicator {
    status: string;
    errorMessage: string;
}

export const RosIndicator = (props: IRosIndicator) => {
    switch (props.status) {
        case 'connected':
            return (
                <Wrapper className="connected">
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
                <Wrapper className="disconnected">
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
                <Wrapper className="error">
                    <i
                        className="material-icons"
                        style={{
                            fontSize: '32px',
                        }}
                    >
                        error_outline
                    </i>
                </Wrapper>
            );
        default:
            return null;
    }
};
