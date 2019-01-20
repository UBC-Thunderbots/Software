/*
 * This file defines the layout for the sidebar title area
 * of the application
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';

import styled from 'SRC/utils/styled-components';

/**
 * Defines the styling of the sidebar title area
 */
const Wrapper = styled.div`
    width: 100%;
    height: 100%;

    display: flex;
    align-items: center;

    color: ${(props) => props.theme.colors.subdued};

    padding: 8px;
`;

interface ISidebarTitleProps {
    /**
     * The sidebar title
     */
    text: string;
}

/**
 * Displays a sidebar title in the correct area of the application
 */
export const SidebarTitle = (props: ISidebarTitleProps) => {
    /*
     * Here, we add the sidebar title in the correct area of the application.
     */
    return ReactDOM.createPortal(
        <Wrapper>{props.text}</Wrapper>,
        document.getElementById('sidebarTitle')!,
    );
};
