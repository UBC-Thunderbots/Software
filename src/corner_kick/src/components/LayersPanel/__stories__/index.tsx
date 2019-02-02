/*
 * This file creates a story for the LayersPanel
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';
import { LayersPanel } from '..';

const stories = storiesOf('Layers Panel', module);

/**
 * Story describing a LayerPanel with some layers defined
 */
stories.add('with layers', () => (
    <StorybookStage width="300px">
        <LayersPanel
            layers={[
                {
                    name: 'Friendly Robots',
                    shapes: [],
                    visible: true,
                },
                {
                    name: 'Enemy Robots',
                    shapes: [],
                    visible: false,
                },
                {
                    name: 'Ball',
                    shapes: [],
                    visible: true,
                },
                {
                    name: 'Field',
                    shapes: [],
                    visible: true,
                },
            ]}
        />
    </StorybookStage>
));

/**
 * Story defining a LayerPanel with no layers — an empty state
 */
stories.add('with empty state', () => (
    <StorybookStage width="300px">
        <LayersPanel layers={[]} />
    </StorybookStage>
));
