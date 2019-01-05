import { dark } from 'ayu';
import * as React from 'react';
import styled from 'styled-components';

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
    color: ${dark.syntax.comment.hex()};
    margin-bottom: 4px;
`;

interface ISettingsEntryProps {
    title: string;
    description: string;
    children: React.ReactNode | React.ReactNodeArray;
}

export const SettingsEntry = (props: ISettingsEntryProps) => {
    return (
        <Wrapper>
            <Title>{props.title}</Title>
            <Description>{props.description}</Description>
            {props.children}
        </Wrapper>
    );
};
