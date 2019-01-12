/**
 * This file describes a React component that displays a single settings
 * entry (for example an input to change the foreground color)
 */

import * as React from 'react';

import styled from 'SRC/utils/styled-components';

const Wrapper = styled.div`
    margin-bottom: 16px;
`;

const Title = styled.div`
    font-size: 14px;
    font-weight: 700;
    margin-bottom: 4px;
`;

const Description = styled.div`
    font-size: 12px;
    color: ${(props) => props.theme.colors.subdued};
    margin-bottom: 4px;
`;

interface ISettingsEntryProps {
    title?: string;
    description?: string;
    children: React.ReactNode | React.ReactNodeArray;
}

export const SettingsEntry = (props: ISettingsEntryProps) => {
    return (
        <Wrapper>
            {props.title !== undefined ? <Title>{props.title}</Title> : null}
            {props.description !== undefined ? (
                <Description>{props.description}</Description>
            ) : null}
            {props.children}
        </Wrapper>
    );
};
