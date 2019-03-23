/*
 * This file creates an story for StorybookStage
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';

const stories = storiesOf('Assets.SVG', module);

const svg = require.context('SRC/assets', true, /.*\.svg$/);

svg.keys().forEach((key: any) => {
    stories.add(key, () => (
        <StorybookStage>
            <img src={svg(key)} style={{ maxWidth: '100%', maxHeight: '100%' }} />
        </StorybookStage>
    ));
});
