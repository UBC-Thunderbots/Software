import styled from 'styled-components';

interface IGridCellProps {
    width?: number;
    height?: number;
    topStart?: number | string;
    leftStart?: number | string;
    topEnd?: number | string;
    leftEnd?: number | string;
    middle?: boolean;
    center?: boolean;
    area?: string;
}

export const GridCell = styled.div<IGridCellProps>`
    height: 100%;
    min-width: 0;
    grid-column-end: ${({ width = 1 }) => `span ${width}`};
    grid-row-end: ${({ height = 1 }) => `span ${height}`};
    ${({ leftStart }) => leftStart && `grid-column-start: ${leftStart}`};
    ${({ topStart }) => topStart && `grid-row-start: ${topStart}`};
    ${({ leftEnd }) => leftEnd && `grid-column-end: ${leftEnd}`};
    ${({ topEnd }) => topEnd && `grid-row-end: ${topEnd}`};
    ${({ center }) => center && `text-align: center`};
    ${({ area }) => area && `grid-area: ${area}`};
    ${/* prettier-ignore */
    ({ middle }) => middle && `
    display: inline-flex;
    flex-flow: column wrap;
    justify-content: center;
    justify-self: stretch;
  `};
`;
