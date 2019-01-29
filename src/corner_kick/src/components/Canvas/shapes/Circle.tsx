import * as React from 'react';
import { IShape } from 'SRC/types';

interface ICircleProps {
    shape: IShape;
}

export const Circle = (props: ICircleProps) => {
    const { data, fill, stroke, stroke_weight } = props.shape;
    const cx = data[0];
    const cy = data[1];
    const r = data[2];

    return (
        <circle
            cx={cx}
            cy={cy}
            r={r}
            style={{
                fill,
                stroke,
                strokeWidth: stroke_weight,
            }}
        />
    );
};
