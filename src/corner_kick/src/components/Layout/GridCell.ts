import styled from 'styled-components';

interface IGridCellProps {
    width?: number;
    height?: number;
    top?: number | string;
    left?: number | string;
    middle?: boolean;
    center?: boolean;
    area?: string;
}

export const GridCell = styled.div<IGridCellProps>`
    height: 100%;
    min-width: 0;
    grid-column-end: ${({ width = 1 }) => `span ${width}`};
    grid-row-end: ${({ height = 1 }) => `span ${height}`};
    ${({ left }) => left && `grid-column-start: ${left}`};
    ${({ top }) => top && `grid-row-start: ${top}`};
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
