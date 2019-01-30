import * as React from 'react';
import { IShape } from 'SRC/types';

interface IEllipseProps {
    shape: IShape;
}

export const Ellipse = (props: IEllipseProps) => {
    const { data, fill, stroke, stroke_weight } = props.shape;
    const cx = data[0];
    const cy = data[1];
    const rx = data[2];
    const ry = data[3];

    return (
        <ellipse
            cx={cx}
            cy={cy}
            rx={rx}
            ry={ry}
            style={{
                fill,
                stroke,
                strokeWidth: stroke_weight,
            }}
        />
    );
};
