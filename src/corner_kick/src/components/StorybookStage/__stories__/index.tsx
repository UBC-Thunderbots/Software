/*
 * This file creates an story for StorybookStage
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from '..';

const stories = storiesOf('Storybook Stage', module);

stories.add('with fixed dimensions', () => (
    <StorybookStage width="300px" height="200px">
        Test content
    </StorybookStage>
));

stories.add('with responsive dimensions', () => (
    <StorybookStage width="30%" height="80%">
        Test content
    </StorybookStage>
));
