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

    let thetaStart = data[3] % (2 * Math.PI);
    let thetaEnd = data[4] % (2 * Math.PI);

    if (thetaStart > thetaEnd) {
        const temp = thetaEnd;
        thetaEnd = thetaStart;
        thetaStart = temp;
    }

    const xStart = cx + r * Math.cos(thetaStart);
    const yStart = cy + r * Math.sin(thetaStart);
    const xEnd = cx + r * Math.cos(thetaEnd);
    const yEnd = cy + r * Math.sin(thetaEnd);

    const largeArc = thetaEnd - thetaStart > Math.PI ? 1 : 0;

    return (
        <>
            <path
                d={`
                    M ${xStart} ${yStart}
                    A ${r} ${r} 0 ${largeArc} 1 ${xEnd} ${yEnd}
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
