import * as React from 'react';
import styled from 'styled-components';

/**
 * Styling for the LogMessage
 */
const StyledWrapper = styled.div`
    padding: 8px;
`;

/**
 * Displays a single log message in our UI
 */
export const LogMessage = (props: {
    message: string;
}) => {
    return (
        <StyledWrapper>
            {props.message}
        </StyledWrapper>
    );
};
