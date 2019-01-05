import { dark } from 'ayu';
import * as React from 'react';
import styled from 'styled-components';

const Input = styled.input`
    width: 100%;
    height: 32px;
    padding: 4px 8px;
    margin: 2px 0;
    border-radius: 4px;
    border: 1px solid ${dark.common.ui.hex()};
    background: transparent;
    color: ${dark.common.fg};
    outline: none;

    &:focus {
        border: 1px solid ${dark.common.accent.hex()};
    }
`;

interface IEditTextProps {
    value?: string;
    onChange?: (event: React.ChangeEvent<HTMLInputElement>) => void;
}

export const EditText = (props: IEditTextProps) => {
    return <Input value={props.value} onChange={props.onChange} />;
};
