import * as React from 'react';
import { IShape } from 'SRC/types';

interface ILineProps {
    shape: IShape;
}

export const Line = (props: ILineProps) => {
    const { data, fill, stroke, stroke_weight } = props.shape;

    const x1 = data[0];
    const y1 = data[1];
    const x2 = data[2];
    const y2 = data[3];

    return (
        <line
            x1={x1}
            y1={y1}
            x2={x2}
            y2={y2}
            style={{
                fill,
                stroke,
                strokeWidth: stroke_weight,
            }}
        />
    );
};
