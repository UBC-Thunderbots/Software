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
                    id: 'friendly_robots',
                    name: 'Friendly Robots',
                    visible: true,
                },
                {
                    id: 'enemy_robots',
                    name: 'Enemy Robots',
                    visible: false,
                },
                {
                    id: 'ball',
                    name: 'Ball',
                    visible: true,
                },
                {
                    id: 'field',
                    name: 'Field',
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
