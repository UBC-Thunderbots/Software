/*
 * This file creates an story for the Logger
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';

import { Logger } from '../Logger';

const stories = storiesOf('Logger', module);

// Generate 1000 mock rosout items
const rosoutFixture = new Array(1000).fill(true).map((_, index) => ({
    file: 'test.c',
    function: 'test.c',
    // We vary the level to show the different styling being applied.
    level: Math.pow(2, index % 5),
    line: 10,
    msg: `Item ${index}`,
    name: 'test',
    topics: ['testTopic'],
}));

stories.add('with 1000 mock log items', () => (
    <StorybookStage width="80%">
        <Logger messages={rosoutFixture} />
    </StorybookStage>
));
