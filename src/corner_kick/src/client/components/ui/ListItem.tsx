import { dark } from 'ayu';
import * as React from 'react';
import styled from 'styled-components';

const Wrapper = styled.div`
    width: 100%;
    height: 24px;

    font-size: 10px;

    cursor: pointer;

    transition: all 0.2s;

    display: flex;
    align-items: center;

    padding: 8px;

    &:hover {
        background: ${dark.ui.selection.bg};
    }
`;

interface IListItem {
    text: string;
}

export const ListItem = (props: IListItem) => {
    return <Wrapper>{props.text}</Wrapper>;
};
