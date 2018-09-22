import * as React from 'react';

import {Circle} from './Circle';
import {Rectangle} from './Rectangle';

interface ISoccerFieldProps {
    x: number;
    y: number;
    lineColor: number;
    lineWidth: number;
    fieldLength: number;
    fieldWidth: number;
    defenseLength: number;
    defenseWidth: number;
    goalWidth: number;
    goalLength: number;
    boundaryWidth: number;
    centerCircleRadius: number;
}

/**
 * A Pixi.js component that renders a soccer field.
 * @param props the properties for this soccer field component
 */
export const SoccerField = (props: ISoccerFieldProps) => {
    const {
        x,
        y,
        lineColor,
        lineWidth,
        fieldLength,
        fieldWidth,
        defenseLength,
        defenseWidth,
        goalWidth,
        goalLength,
        boundaryWidth,
        centerCircleRadius,
    } = props;

    return (
        <Rectangle
            name='complete-field'
            x={x}
            y={y}
            width={fieldWidth + boundaryWidth * 2}
            height={fieldLength + boundaryWidth * 2}
            stroke={0X000000}
            strokeWidth={lineWidth}
        >
            <Rectangle
                name='goal-top'
                x={boundaryWidth + (fieldWidth - goalWidth) / 2}
                y={boundaryWidth - goalLength}
                width={goalWidth}
                height={goalLength}
                stroke={lineColor}
                strokeWidth={lineWidth}
            />
            <Rectangle
                name='goal-bottom'
                x={boundaryWidth + (fieldWidth - goalWidth) / 2}
                y={boundaryWidth + fieldLength}
                width={goalWidth}
                height={goalLength}
                stroke={lineColor}
                strokeWidth={lineWidth}
            />
            <Rectangle
                name='playing-field'
                x={boundaryWidth}
                y={boundaryWidth}
                width={fieldWidth}
                height={fieldLength}
                stroke={lineColor}
                strokeWidth={lineWidth}
            >
                <Rectangle
                    name='defense-top'
                    x={(fieldWidth - defenseWidth) / 2}
                    y={0}
                    width={defenseWidth}
                    height={defenseLength}
                    stroke={lineColor}
                    strokeWidth={lineWidth}
                />
                <Rectangle
                    name='defense-bottom'
                    x={(fieldWidth - defenseWidth) / 2}
                    y={fieldLength - defenseLength}
                    width={defenseWidth}
                    height={defenseLength}
                    stroke={lineColor}
                    strokeWidth={lineWidth}
                />

                <Rectangle
                    name='center-line'
                    x={0}
                    y={(fieldLength - lineWidth) / 2}
                    width={fieldWidth}
                    height={lineWidth}
                    fill={lineColor}
                />

                <Circle
                    name='center-circle'
                    x={fieldWidth / 2}
                    y={fieldLength / 2}
                    radius={centerCircleRadius}
                    stroke={lineColor}
                    strokeWidth={lineWidth}
                />
            </Rectangle>
        </Rectangle>
    );
};