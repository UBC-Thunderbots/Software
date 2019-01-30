import * as React from 'react';
import { IShape } from 'SRC/types';

interface IPolygonProps {
    shape: IShape;
}

export const Polygon = (props: IPolygonProps) => {
    const { data, fill, stroke, stroke_weight } = props.shape;

    return (
        <>
            <path
                d={`
                    M ${data[0]} ${data[1]}
                    ${data.slice(2).reduce((prev, curr, index) => {
                        if (index % 2 === 0) {
                            return prev + `L ${curr} `;
                        } else {
                            return prev + `${curr}\n`;
                        }
                    }, '')}
                    Z
            `}
                style={{
                    fill,
                    stroke,
                    strokeWidth: stroke_weight,
                }}
            />
        </>
    );
};
