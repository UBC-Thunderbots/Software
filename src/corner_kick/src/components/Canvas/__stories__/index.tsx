/*
 * This file creates an example story for Storybook
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';

import { Canvas } from '..';

import arcs from './fixtures/arcs.json';
import ellipses from './fixtures/ellipses.json';
import lines from './fixtures/lines.json';
import poly from './fixtures/poly.json';
import rect from './fixtures/rect.json';

const stories = storiesOf('Shape Canvas', module);

stories.add('with arcs', () => (
    <StorybookStage width="60%" height="80%">
        <Canvas layers={arcs} worldWidth={9.7} worldHeight={3.1} />
    </StorybookStage>
));

stories.add('with ellipses', () => (
    <StorybookStage width="60%" height="80%">
        <Canvas layers={ellipses} worldWidth={9.3} worldHeight={3.1} />
    </StorybookStage>
));

stories.add('with rects', () => (
    <StorybookStage width="60%" height="80%">
        <Canvas layers={rect} worldWidth={10} worldHeight={4.4} />
    </StorybookStage>
));

stories.add('with lines', () => (
    <StorybookStage width="60%" height="80%">
        <Canvas layers={lines} worldWidth={9.4} worldHeight={3.2} />
    </StorybookStage>
));

stories.add('with poly', () => (
    <StorybookStage width="60%" height="80%">
        <Canvas layers={poly} worldWidth={8} worldHeight={4} />
    </StorybookStage>
));
