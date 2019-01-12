/**
 * This file defines a field in the visualizer
 */

import * as React from 'react';

export const Field = (props: {
    fieldWidth: number;
    fieldLength: number;
    defenseWidth: number;
    defenseLength: number;
    goalWidth: number;
    boundaryWidth: number;
    centerCircleRadius: number;
    children?: React.ReactNode | React.ReactNodeArray;
}) => {
    return (
        <g
            width={props.fieldLength + 2 * props.boundaryWidth}
            height={props.fieldWidth + 2 * props.boundaryWidth}
        >
            <rect
                x={props.boundaryWidth}
                y={props.boundaryWidth}
                width={props.fieldLength}
                height={props.fieldWidth}
            />
            <g transform={`translate(${props.boundaryWidth}, ${props.boundaryWidth})`}>
                <rect
                    y={(props.fieldWidth - props.defenseWidth) / 2}
                    width={props.defenseLength}
                    height={props.defenseWidth}
                />
                <rect
                    x={props.fieldLength - props.defenseLength}
                    y={(props.fieldWidth - props.defenseWidth) / 2}
                    width={props.defenseLength}
                    height={props.defenseWidth}
                />
                <line
                    x1={props.fieldLength / 2}
                    x2={props.fieldLength / 2}
                    y2={props.fieldWidth}
                />
                <circle
                    cx={props.fieldLength / 2}
                    cy={props.fieldWidth / 2}
                    r={props.centerCircleRadius}
                />
                {props.children}
            </g>
        </g>
    );
};
