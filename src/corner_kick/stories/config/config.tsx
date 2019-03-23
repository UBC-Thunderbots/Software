/*
 * This file initializes Storybook, and adds
 * styling decorators
 */

import { withTests } from '@storybook/addon-jest';
import { addDecorator, addParameters, configure } from '@storybook/react';
import * as React from 'react';

import { Theme } from 'SRC/style/Theme';

import theme from './theme';

import results from '../../.jest-test-results.json';

// automatically import all files in the stories directory
const req = (require as any).context('..', true, /.*\.story\.tsx?$/);

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

addDecorator(withTests({
    results,
}) as any);

// Init Storybook
configure(loadStories, module);
