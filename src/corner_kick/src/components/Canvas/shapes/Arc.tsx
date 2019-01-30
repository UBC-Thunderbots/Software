import * as React from 'react';
import { IShape } from 'SRC/types';

interface IArcProps {
    shape: IShape;
}

export const Arc = (props: IArcProps) => {
    const { data, fill, stroke, stroke_weight } = props.shape;
    const cx = data[0];
    const cy = data[1];
    const r = data[2];
    const thetaStart = data[3] % 360;
    const thetaEnd = data[4] % 360;

    const xStart = cx + r * Math.cos((thetaStart * Math.PI) / 180);
    const yStart = cy + r * Math.sin((thetaStart * Math.PI) / 180);
    const xEnd = cx + r * Math.cos((thetaEnd * Math.PI) / 180);
    const yEnd = cy + r * Math.sin((thetaEnd * Math.PI) / 180);

    return (
        <path
            d={`
            M ${xStart} ${yStart}
            A ${r} ${r} 0 ${thetaEnd - thetaStart > 180 ? 1 : 0} 1 ${xEnd} ${yEnd}
            `}
            style={{
                fill,
                stroke,
                strokeWidth: stroke_weight,
            }}
        />
    );
};
