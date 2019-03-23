/*
 * This file creates an story for StorybookStage
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';

const stories = storiesOf('Assets.PNG', module);

const png = require.context('SRC/assets', true, /.*\.png$/);

png.keys().forEach((key: any) => {
    stories.add(key, () => (
        <StorybookStage>
            <img src={png(key)} style={{ maxWidth: '100%', maxHeight: '100%' }} />
        </StorybookStage>
    ));
});
