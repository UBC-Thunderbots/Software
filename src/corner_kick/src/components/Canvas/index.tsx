import * as React from 'react';

import { ILayer } from 'SRC/types';
import styled from 'SRC/utils/styled-components';

import { Arc } from './shapes/Arc';
import { Ellipse } from './shapes/Ellipse';
import { Line } from './shapes/Line';
import { Polygon } from './shapes/Polygon';
import { Rect } from './shapes/Rect';

const Wrapper = styled.svg`
    & * {
        fill: transparent;
        stroke: transparent;
    }
`;

interface ICanvasProps {
    startX?: number;
    startY?: number;
    worldWidth: number;
    worldHeight: number;
    layers: ILayer[];
}

export const Canvas = (props: ICanvasProps) => {
    const { layers, startX, startY, worldWidth, worldHeight } = props;
    return (
        <Wrapper
            width="100%"
            height="100%"
            viewBox={[startX || 0, startY || 0, worldWidth, worldHeight].join(' ')}
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
                                case 'poly':
                                    return (
                                        <Polygon
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
