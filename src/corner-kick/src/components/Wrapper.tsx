import * as React from 'react';
import styled from 'styled-components';

export const StyledWrapper = styled.div`
    display: grid;
    grid-template-rows: 50px 1fr;
`;

export const Wrapper = (props: {children: React.ReactNode}) => {
    return (
        <StyledWrapper>
            {props.children}
        </StyledWrapper>
    )
}