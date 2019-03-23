/*
 * This file creates an story for StorybookStage
 */

import { storiesOf } from '@storybook/react';

import * as React from 'react';

import CornerKickReadme from '../../README.md';

const stories = storiesOf('General', module).addParameters({
    notes: {
        markdown: CornerKickReadme,
    },
});

stories.add('Corner Kick dependencies', () => <div />, {
    jest: ['nodeModules.test.ts'],
});
