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
                    id: 1,
                    visible: true,
                },
                {
                    id: 2,
                    visible: false,
                },
                {
                    id: 3,
                    visible: true,
                },
                {
                    id: 4,
                    visible: true,
                },
            ]}
            toggleVisibility={(id) => alert(`Layer ${id} had its visibility toggled`)}
        />
    </StorybookStage>
));

/**
 * Story defining a LayerPanel with no layers — an empty state
 */
stories.add('with empty state', () => (
    <StorybookStage width="300px">
        <LayersPanel
            layers={[]}
            toggleVisibility={() => {
                //
            }}
        />
    </StorybookStage>
));
