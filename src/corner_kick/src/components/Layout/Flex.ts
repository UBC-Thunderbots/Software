/***
 * This file specifies the Flex component, which simplifies the creation of
 * a common UI layout element
 */
import styled from 'SRC/utils/styled-components';

interface IFlexProps {
    width?: string;
    height?: string;
    padding?: string;
    margin?: string;
    cursor?: string;
    flexFlow?: string;
    flex?: string;
    justifyContent?: string;
    alignItems?: string;
}

/**
 * Flex creates a flexbox HTML div element
 */
export const Flex = styled.div<IFlexProps>`
    display: flex;
    width: ${({ width = 'auto' }) => width};
    height: ${({ height = 'auto' }) => height};
    padding: ${({ padding = '0' }) => padding};
    margin: ${({ margin = '0' }) => margin};
    ${({ flexFlow }) => flexFlow && `flex-flow: ${flexFlow}`};
    ${({ flex }) => flex && `flex: ${flex}`};
    ${({ justifyContent }) => justifyContent && `justify-content: ${justifyContent}`};
    ${({ alignItems }) => alignItems && `align-items: ${alignItems}`};
    ${({ cursor }) => cursor && `cursor: ${cursor}`};
`;
