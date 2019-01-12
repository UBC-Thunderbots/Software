/**
 * This file defines a React component used to display a category
 */

import * as React from 'react';
import { Link } from 'react-router-dom';

import styled from 'SRC/utils/styled-components';

const Wrapper = styled.div`
    width: 100%;
    height: 48px;

    font-size: 12px;
    color: ${(props) => props.theme.colors.fg};
    text-transform: uppercase;
    font-weight: 700;

    cursor: pointer;

    transition: all 0.2s;

    display: flex;
    align-items: center;

    padding: 8px 8px 8px 16px;

    &:hover {
        background: ${(props) => props.theme.colors.selected};
    }
`;

const Icon = styled.i`
    font-size: 22px;
    margin-right: 8px;
`;

interface ICategoryTitle {
    text: string;
    icon?: string;
    onClick?: (e: React.MouseEvent) => void;
    link?: string;
}

/**
 * Displays a category, with an icon and text
 */
export const CategoryItem = (props: ICategoryTitle) => {
    const LinkIfNeeded = (props.link !== undefined ? Link : 'div') as any;

    return (
        <LinkIfNeeded to={props.link}>
            <Wrapper onClick={props.onClick}>
                {props.icon !== undefined ? (
                    <Icon className="material-icons">{props.icon}</Icon>
                ) : null}
                {props.text}
                <Icon className="material-icons" style={{ marginLeft: 'auto' }}>
                    keyboard_arrow_right
                </Icon>
            </Wrapper>
        </LinkIfNeeded>
    );
};
