/*
 * This file initializes Storybook, adds support for Percy (https://percy.io/), and adds
 * styling decorators
 */

import createPercyAddon from '@percy-io/percy-storybook';
import { addDecorator, configure, getStorybook, setAddon } from '@storybook/react';
import * as React from 'react';

import { Theme } from 'SRC/style/Theme';

// Add Percy support
const { percyAddon, serializeStories } = createPercyAddon();
setAddon(percyAddon);

// automatically import all files in the __stories__ directory
const req = (require as any).context('../src', true, /__stories__\/.*.tsx$/);

// Load all stories from each file
function loadStories() {
    req.keys().forEach(req);
}

// Add application styling to all stories
const StyleDecorator = (storyFn: () => JSX.Element) => <Theme>{storyFn()}</Theme>;
addDecorator(StyleDecorator);

// Init Storybook
configure(loadStories, module);

// Send all info to Percy
serializeStories(getStorybook);
