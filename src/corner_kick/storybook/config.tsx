import createPercyAddon from '@percy-io/percy-storybook';
import { addDecorator, configure, getStorybook, setAddon } from '@storybook/react';
import * as React from 'react';

import { Theme } from '../src/style/Theme';

// Add Percy support
const { percyAddon, serializeStories } = createPercyAddon();
setAddon(percyAddon);

// automatically import all files in the __stories__ directory
const req = (require as any).context('../src', true, /__stories__\/.*.tsx$/);

function loadStories() {
    req.keys().forEach(req);
}

const StyleDecorator = (storyFn: () => JSX.Element) => <Theme>{storyFn()}</Theme>;
addDecorator(StyleDecorator);

configure(loadStories, module);

// Send all info to Percy
serializeStories(getStorybook);
