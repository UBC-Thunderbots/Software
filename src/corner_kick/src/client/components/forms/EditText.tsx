import * as React from 'react';

import styled from 'SRC/utils/styled-components';

const Input = styled.input`
    width: 100%;
    height: 32px;
    padding: 4px 8px;
    margin: 2px 0;
    border-radius: 4px;
    border: 1px solid ${(props) => props.theme.colors.subdued};
    background: transparent;
    color: ${(props) => props.theme.colors.fg};
    outline: none;

    &:focus {
        border: 1px solid ${(props) => props.theme.colors.accent};
    }
`;

interface IEditTextProps {
    value?: string;
    onChange?: (event: React.ChangeEvent<HTMLInputElement>) => void;
}

export const EditText = (props: IEditTextProps) => {
    return <Input value={props.value} onChange={props.onChange} />;
};
