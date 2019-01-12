/**
 * Defines the main logo of the application
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';

import styled from 'SRC/utils/styled-components';

// We import the logo
import logo from 'SRC/assets/logo.svg';

/**
 * Defines the styling for the logo
 */
const Wrapper = styled.div`
    width: 56px;
    height: 56px;

    display: flex;
    justify-content: center;
    align-items: center;

    border-bottom: 1px solid ${(props) => props.theme.colors.border};
`;

/**
 * Displays the application logo in the sidebar control area.
 */
export const Logo = () => {
    /*
     * Here, we create the layout for the logo and add it to the sidebar
     * control area.
     */
    return ReactDOM.createPortal(
        <Wrapper>
            <img src={logo} width={32} height={32} draggable={false} />
        </Wrapper>,
        document.getElementById('sidebarControl')!,
    );
};
