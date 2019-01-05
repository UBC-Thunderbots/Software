/**
 * @fileoverview Defines the UI for a page icon, which will be added to the
 * sidebar control area of the application.
 */

import { dark } from 'ayu';
import * as React from 'react';
import * as ReactDOM from 'react-dom';
import styled from 'styled-components';

/** Defines the styling of the page icon */
const Wrapper = styled.div`
    width: 55px;
    height: 55px;

    display: flex;
    justify-content: center;
    align-items: center;

    color: ${dark.syntax.comment};

    cursor: pointer;
    transition: all 0.2s;

    &:hover {
        background: ${dark.ui.selection.bg};
    }
`;

/** Defines the styling of the icon itself */
const Icon = styled.i`
    font-size: 32px;
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
        <Wrapper
            title={props.title}
            style={{ background: props.active ? dark.ui.selection.bg.hex() : undefined }}
        >
            <Icon
                className="material-icons"
                style={{
                    color: props.active ? dark.common.accent.hex() : undefined,
                }}
            >
                {props.icon}
            </Icon>
        </Wrapper>,
        document.getElementById('sidebarControl')!,
    );
};
