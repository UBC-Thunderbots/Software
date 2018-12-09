/**
 * @fileoverview React Component that displays a single
 * log message.
 */

import * as React from 'react';
import styled from 'styled-components';

/**
 * @description Styling for the LogMessage.
 */
const StyledWrapper = styled.div`
    padding: 8px;
`;

/**
 * @description Displays a single log message in our UI.
 * @param props Accepts a message string.
 */
export const LogMessage = (props: { message: string }) => {
    return <StyledWrapper>{props.message}</StyledWrapper>;
};
