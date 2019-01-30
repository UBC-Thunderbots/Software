import * as React from 'react';

import { ILayer } from 'SRC/types';
import styled from 'SRC/utils/styled-components';

import { Arc } from './shapes/Arc';
import { Ellipse } from './shapes/Ellipse';
import { Line } from './shapes/Line';
import { Rect } from './shapes/Rect';

const Wrapper = styled.svg`
    & * {
        fill: transparent;
        stroke: transparent;
    }
`;

interface ICanvasProps {
    layers: ILayer[];
}

export const Canvas = (props: ICanvasProps) => {
    const { layers } = props;
    return (
        <Wrapper
            width="100%"
            height="100%"
            viewBox={[0, 0, 12.6, 9.6].join(' ')}
            xmlns="http://www.w3.org/2000/svg"
        >
            {layers
                .slice(0)
                .reverse()
                .map((layer) => {
                    if (layer.visible) {
                        return layer.shapes.reverse().map((shape, index) => {
                            switch (shape.type) {
                                case 'ellipse':
                                    return (
                                        <Ellipse
                                            key={`${layer.name}${index}`}
                                            shape={shape}
                                        />
                                    );
                                case 'rect':
                                    return (
                                        <Rect
                                            key={`${layer.name}${index}`}
                                            shape={shape}
                                        />
                                    );
                                case 'line':
                                    return (
                                        <Line
                                            key={`${layer.name}${index}`}
                                            shape={shape}
                                        />
                                    );
                                case 'arc':
                                    return (
                                        <Arc
                                            key={`${layer.name}${index}`}
                                            shape={shape}
                                        />
                                    );
                                default:
                                    return null;
                            }
                        });
                    } else {
                        return null;
                    }
                })}
        </Wrapper>
    );
};
