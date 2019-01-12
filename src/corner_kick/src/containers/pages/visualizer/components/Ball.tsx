/**
 * This file defines a ball in the visualizer
 */

import * as React from 'react';

import styled from 'SRC/utils/styled-components';

const Circle = styled.circle`
    stroke: transparent;
    fill: ${(props) => props.theme.colors.orange};
`;

export const Ball = (props: {
    x: number;
    y: number;
    dX: number;
    dY: number;
    fieldLength: number;
    fieldWidth: number;
}) => {
    return (
        <Circle
            cx={props.fieldLength / 2 + props.x}
            cy={props.fieldWidth / 2 + props.y}
            r={0.1}
        />
    );
};
