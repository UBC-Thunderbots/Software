import { dark } from 'ayu';
import * as React from 'react';

export const Ball = (props: {
    x: number;
    y: number;
    dX: number;
    dY: number;
    fieldLength: number;
    fieldWidth: number;
}) => {
    return (
        <circle
            cx={props.fieldLength / 2 + props.x}
            cy={props.fieldWidth / 2 + props.y}
            r={0.1}
            stroke="transparent"
            fill={dark.syntax.markup.hex()}
        />
    );
};
