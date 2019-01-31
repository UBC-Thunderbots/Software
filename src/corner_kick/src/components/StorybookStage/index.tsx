/**
 * This file defines a UI element that centers and bounds the item we want
 * to display in a Storybook story
 */

import * as React from 'react';
import styled from 'SRC/utils/styled-components';

/**
 * Provides centering capabilities
 */
const Wrapper = styled.div`
    width: 100vw;
    height: 100vh;

    display: flex;
    justify-content: center;
    align-items: center;

    background: ${(props) => props.theme.colors.selected};
`;

/**
 * Provides bounding capabilities
 */
const Content = styled('div')<{ width?: string; height?: string }>`
    width: ${(props) => props.width};
    height: ${(props) => props.height};
    border: 1px solid ${(props) => props.theme.colors.border};
    background: ${(props) => props.theme.colors.bg};
`;

interface IStorybookStageProps {
    width?: string;
    height?: string;
    children: React.ReactNode;
}

/**
 * UI element allowing us to center and bound what we want to display
 * in a Storybook story
 */
export const StorybookStage = (props: IStorybookStageProps) => {
    return (
        <Wrapper>
            <Content width={props.width} height={props.height}>
                {props.children}
            </Content>
        </Wrapper>
    );
};
