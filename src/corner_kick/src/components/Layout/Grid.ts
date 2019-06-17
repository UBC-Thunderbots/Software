import styled from 'SRC/utils/styled-components';

interface IGridProps {
    width?: string;
    height?: string;
    padding?: string;
    margin?: string;
    cursor?: string;
    columns?: string | number;
    gap?: string;
    columnGap?: string;
    rowGap?: string;
    minRowHeight?: string;
    flow?: string;
    rows?: string | number;
    areas?: string[];
}

const autoRows = ({ minRowHeight = '20px' }) => `minmax(${minRowHeight}, auto)`;

const frGetter = (value: string | number) =>
    typeof value === 'number' ? `repeat(${value}, 1fr)` : value;

const gap = ({ gap = '5px' }) => gap;

const flow = ({ flow = 'row' }) => flow;

const formatAreas = (areas: string[]) => areas.map((area) => `"${area}"`).join(' ');

export const Grid = styled.div<IGridProps>`
    display: grid;
    width: ${({ width = 'auto' }) => width};
    height: ${({ height = 'auto' }) => height};
    padding: ${({ padding = '0' }) => padding};
    margin: ${({ margin = '0' }) => margin};
    grid-auto-flow: ${flow};
    grid-auto-rows: ${autoRows};
    ${({ rows }) => rows && `grid-template-rows: ${frGetter(rows)}`};
    grid-template-columns: ${({ columns = 12 }) => frGetter(columns)};
    grid-gap: ${gap};
    ${({ columnGap }) => columnGap && `column-gap: ${columnGap}`};
    ${({ rowGap }) => rowGap && `row-gap: ${rowGap}`};
    ${({ areas }) => areas && `grid-template-areas: ${formatAreas(areas)}`};
    ${({ cursor }) => cursor && `cursor: ${cursor}`};
`;
