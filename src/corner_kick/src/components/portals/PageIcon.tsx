/**
 * Defines the UI for a page icon, which will be added to the
 * sidebar control area of the application.
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';

import styled from 'SRC/utils/styled-components';

/** Defines the styling of the page icon */
const Wrapper = styled.div`
    width: 55px;
    height: 55px;

    display: flex;
    justify-content: center;
    align-items: center;

    color: ${(props) => props.theme.colors.subdued};

    cursor: pointer;
    transition: all 0.2s;

    &:hover {
        background: ${(props) => props.theme.colors.selected};
    }

    &.active {
        background: ${(props) => props.theme.colors.selected};
    }
`;

/** Defines the styling of the icon itself */
const Icon = styled.i`
    font-size: 32px;

    .active & {
        color: ${(props) => props.theme.colors.accent};
    }
`;

interface IFooterItemProps {
    /** Indicate if the page is currently active */
    active: boolean;

    /** Indicate the icon to use */
    icon: string;

    /** Indicate the text to diplay on hover */
    title: string;
}

/**
 * Displays an icon for a specific page of the application
 * @param props the props for this React component
 */
export const PageIcon = (props: IFooterItemProps) => {
    /*
     * Here, we define the layout of the page icon and add it to the sidebar control
     * area of the application.
     */
    return ReactDOM.createPortal(
        <Wrapper title={props.title} className={props.active ? 'active' : ''}>
            <Icon className="material-icons">{props.icon}</Icon>
        </Wrapper>,
        document.getElementById('sidebarControl')!,
    );
};
