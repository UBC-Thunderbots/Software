/*
 * This file creates an example story for Storybook
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';

import { Canvas } from '..';

const stories = storiesOf('Shape Canvas', module);

stories.add('with an example game', () => (
    <StorybookStage width="60%" height="80%">
        <Canvas
            layers={[
                {
                    name: 'Ball',
                    shapes: [
                        {
                            data: [6.3, 4.8, 0.06],
                            fill: 'orange',
                            type: 'ellipse',
                        },
                    ],
                    visible: true,
                },
                {
                    name: 'Friendly Robots',
                    shapes: [
                        {
                            data: [2, 3.3, 0.24],
                            fill: 'red',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                        {
                            data: [2.5, 6.3, 0.24],
                            fill: 'red',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                        {
                            data: [0.5, 4.3, 0.24],
                            fill: 'red',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                        {
                            data: [10.5, 7.3, 0.24],
                            fill: 'red',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                        {
                            data: [6.5, 8.3, 0.24],
                            fill: 'red',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                    ],
                    visible: true,
                },
                {
                    name: 'Enemy Robots',
                    shapes: [
                        {
                            data: [6, 3, 0.24],
                            fill: 'blue',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                        {
                            data: [2, 3.8, 0.24],
                            fill: 'blue',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                        {
                            data: [11.5, 4.8, 0.24],
                            fill: 'blue',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                        {
                            data: [9, 8.8, 0.24],
                            fill: 'blue',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                        {
                            data: [6, 6.8, 0.24],
                            fill: 'blue',
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'ellipse',
                        },
                    ],
                    visible: true,
                },
                {
                    name: 'Field',
                    shapes: [
                        {
                            data: [6.3, 4.8, 0.5, 0.5],
                            stroke: 'white',
                            stroke_weight: 0.01,
                            type: 'ellipse',
                        },
                        {
                            data: [6.3, 0.3, 6.3, 9.3],
                            stroke: 'white',
                            stroke_weight: 0.01,
                            type: 'line',
                        },
                        {
                            data: [0.1, 4.2, 0.2, 1.2],
                            stroke: 'white',
                            stroke_weight: 0.01,
                            type: 'rect',
                        },
                        {
                            data: [12.3, 4.2, 0.2, 1.2],
                            stroke: 'white',
                            stroke_weight: 0.01,
                            type: 'rect',
                        },
                        {
                            data: [0.3, 3.6, 1.2, 2.4],
                            stroke: 'white',
                            stroke_weight: 0.01,
                            type: 'rect',
                        },
                        {
                            data: [11.1, 3.6, 1.2, 2.4],
                            stroke: 'white',
                            stroke_weight: 0.01,
                            type: 'rect',
                        },
                        {
                            data: [0.3, 0.3, 12, 9],
                            stroke: 'white',
                            stroke_weight: 0.01,
                            type: 'rect',
                        },
                    ],
                    visible: true,
                },
            ]}
        />
    </StorybookStage>
));
