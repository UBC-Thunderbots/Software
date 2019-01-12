/**
 * React element that allows for text editing
 */

import * as React from 'react';

import styled from 'SRC/utils/styled-components';

const Wrapper = styled.div`
    position: relative;
    width: 100%;
    height: 32px;
    margin: 2px 0;
`;

const Input = styled.input`
    width: 100%;
    height: 100%;
    padding: 4px 8px;
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

/**
 * Enables the user to edit a piece of text.
 *
 * This element can be controlled by React or uncontrolled, depending on
 * whether the value prop is defined or not.
 */
export const EditText = (props: IEditTextProps) => {
    return (
        <Wrapper>
            <Input value={props.value} onChange={props.onChange} />
        </Wrapper>
    );
};
