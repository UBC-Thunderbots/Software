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
                            data: [1, 1, 0.5, -70, 30],
                            stroke: 'white',
                            stroke_weight: 0.03,
                            type: 'arc',
                        },
                    ],
                    visible: true,
                },
            ]}
        />
    </StorybookStage>
));
