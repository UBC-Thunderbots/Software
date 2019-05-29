/*
 * This file creates a story for the SpritesheetManager
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';

import { expectedSpritesheetFormat } from '../__fixtures__/spritesheet';
import { SpritesheetManager } from '../spritesheetManager';

const stories = storiesOf('Spritesheet Generator', module);

const spritesheetGenerator = new SpritesheetManager(expectedSpritesheetFormat);

stories.add(
    'with an example spritesheet',
    () => (
        <StorybookStage height="80%">
            <img height="100%" src={spritesheetGenerator.getSpritesheetPNG()} />
        </StorybookStage>
    ),
    {
        notes: {
            markdown: `
### What you should see (from left to right, top to bottom):

1. A rectangle
1. A circle
1. An arc stroke, from 0 degrees (defined as straight left) to 180 degrees (clockwise)
1. A triangle stroke, on the bottom quadrant
1. A line from top left to bottom right of the screen
1. A filled arc from -90 degrees to 180 degrees

All shapes should be white and should take most of their section.
            `,
        },
    },
);
