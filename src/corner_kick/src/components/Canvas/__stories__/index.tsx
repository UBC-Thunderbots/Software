/*
 * This file creates a story for the Canvas
 */

import { color, number } from '@storybook/addon-knobs';
import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';

import { Canvas } from '..';

const stories = storiesOf('Shape Canvas', module);

// We define the options for the various dimension related knobs
const dimenOptions = {
    max: 10,
    min: 0,
    range: true,
    step: 0.5,
};

// We define the options for the various style related knobs
const weightOptions = {
    max: 1,
    min: 0,
    range: true,
    step: 0.05,
};
stories.add('with rects', () => {
    // Some of the parameters of this fixture can be changed by the user
    const fixture = [
        {
            name: 'Rect',
            shapes: [
                {
                    data: [
                        number('x', 1, dimenOptions, 'dimensions'),
                        number('y', 1, dimenOptions, 'dimensions'),
                        number('width', 1, dimenOptions, 'dimensions'),
                        number('height', 1, dimenOptions, 'dimensions'),
                    ],
                    fill: color('fill', 'transparent', 'style'),
                    stroke: color('stroke', 'white', 'style'),
                    stroke_weight: number('stroke_weight', 0.05, weightOptions, 'style'),
                    type: 'rect',
                },
            ],
            visible: true,
        },
    ];

    return (
        <StorybookStage width="60%" height="80%">
            <Canvas layers={fixture} worldWidth={10} worldHeight={10} />
        </StorybookStage>
    );
});
