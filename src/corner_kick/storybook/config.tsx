/*
 * This file initializes Storybook, and adds
 * styling decorators
 */

import { addDecorator, addParameters, configure } from '@storybook/react';
import * as React from 'react';

import { Theme } from 'SRC/style/Theme';

import theme from './theme';

import '@blueprintjs/core/lib/css/blueprint.css';
import '@blueprintjs/icons/lib/css/blueprint-icons.css';

// automatically import all files in the __stories__ directory
const req = (require as any).context('../src', true, /__stories__\/.*.tsx$/);

// Load all stories from each file
function loadStories() {
    req.keys().forEach(req);
}

// Add application styling to all stories
const StyleDecorator = (storyFn: () => JSX.Element) => <Theme>{storyFn()}</Theme>;
addDecorator(StyleDecorator);

addParameters({
    options: {
        theme,
    },
});

// Init Storybook
configure(loadStories, module);
