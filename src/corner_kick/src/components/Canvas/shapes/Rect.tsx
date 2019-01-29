import * as React from 'react';
import { IShape } from 'SRC/types';

interface IRectProps {
    shape: IShape;
}

export const Rect = (props: IRectProps) => {
    const { data, fill, stroke, stroke_weight } = props.shape;

    const x = data[0];
    const y = data[1];
    const width = data[2];
    const height = data[3];

    return (
        <rect
            x={x}
            y={y}
            width={width}
            height={height}
            style={{
                fill,
                stroke,
                strokeWidth: stroke_weight,
            }}
        />
    );
};
