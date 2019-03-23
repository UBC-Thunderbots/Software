/*
 * This file creates an story for StorybookStage
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { App } from 'SRC/App';

import { InjectPortals } from './InjectPortals';

import CornerKickReadme from '../../README.md';

const stories = storiesOf('General', module).addParameters({
    notes: {
        markdown: CornerKickReadme,
    },
});

stories.add('Full application', () => (
    <InjectPortals>
        <App />
    </InjectPortals>
));
