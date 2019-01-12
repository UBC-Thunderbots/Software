/**
 * This file defines a UI component called FooterIndicator. This
 * adds a small indicator in the footer area of the application.
 */

import * as React from 'react';
import * as ReactDOM from 'react-dom';
import { Link } from 'react-router-dom';

import styled from 'SRC/utils/styled-components';

/** Main styling for the indicator */
const Wrapper = styled.div`
    height: 100%;
    margin: 0 4px;
    padding: 0 2px;

    display: flex;
    align-items: center;

    color: ${(props) => props.theme.colors.subdued};
    font-size: 12px;
    text-decoration: none !important;

    cursor: pointer;
    transition: all 0.2s;

    &:hover {
        background: ${(props) => props.theme.colors.selected};
    }
`;

/** Styling for the icon */
const Icon = styled.i`
    font-size: 14px;
    margin-right: 2px;
`;

interface IFooterItemProps {
    /** Icon displayed on the left of the indicator */
    icon: string;

    /** Text displayed in the indicator */
    text: string;

    /** Display the indicator on the right or left of the footer */
    direction: 'left' | 'right';

    /** Text displayed on hover */
    title?: string;

    /** Link for this indicator */
    link?: string;

    /** Set a custom color for the text, default is grey */
    color?: string;
}

/**
 * Displays a small indicator on the right or left of the footer area.
 * @param props the props for this React component
 */
export const FooterIndicator = (props: IFooterItemProps) => {
    // Check if the link parameter is defined in the props and create a link if needed
    const LinkIfNeeded = (props.link !== undefined ? Link : 'div') as any;

    /*
     * Here, we create the layout of the indicator (text + icon)
     * and add it to the footer area (left or right) via a React Portal.
     */
    return ReactDOM.createPortal(
        <LinkIfNeeded to={props.link}>
            <Wrapper title={props.title} style={{ color: props.color }}>
                <Icon className="material-icons">{props.icon}</Icon>
                {props.text}
            </Wrapper>
        </LinkIfNeeded>,
        document.getElementById(
            props.direction === 'left' ? 'footerLeft' : 'footerRight',
        )!,
    );
};
