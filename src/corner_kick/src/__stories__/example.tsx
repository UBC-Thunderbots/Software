/*
 * This file creates an example story for Storybook
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

const stories = storiesOf('Hello World', module);

stories.add('Hello World!', () => <div>Hello World!</div>);
stories.add('Testing!', () => <div>Testing!</div>);
