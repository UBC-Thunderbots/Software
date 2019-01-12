import * as React from 'react';
import Scrollbars from 'react-custom-scrollbars';

import styled from 'SRC/utils/styled-components';

const Wrapper = styled.div`
    width: 100%;
    display: flex;
    justify-content: center;
`;

const InnerWrapper = styled.div`
    width: 100%;
    max-width: 600px;
    padding: 16px;
`;

export const SettingsCategory = (props: {
    children: React.ReactNode | React.ReactNodeArray;
}) => {
    return (
        <Scrollbars>
            <Wrapper>
                <InnerWrapper>{props.children}</InnerWrapper>
            </Wrapper>
        </Scrollbars>
    );
};
