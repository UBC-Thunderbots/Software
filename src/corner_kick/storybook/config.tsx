/*
 * This file initializes Storybook, and adds
 * styling decorators
 */

import { withKnobs } from '@storybook/addon-knobs';
import { addDecorator, configure } from '@storybook/react';
import * as React from 'react';

import { Theme } from 'SRC/style/Theme';

// automatically import all files in the __stories__ directory
const req = (require as any).context('../src', true, /__stories__\/.*.tsx$/);

// Load all stories from each file
function loadStories() {
    req.keys().forEach(req);
}

// Add application styling to all stories
const StyleDecorator = (storyFn: () => JSX.Element) => <Theme>{storyFn()}</Theme>;
addDecorator(StyleDecorator);

// Add Storybook knobs
addDecorator(withKnobs);

// Init Storybook
configure(loadStories, module);
