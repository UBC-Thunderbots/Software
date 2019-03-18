/**
 * This file specifies the theme of Storybook
 */

import { create } from '@storybook/theming';

import logo from '../src/assets/logo.svg';

export default create({
    base: 'light',

    brandImage: logo,
    brandTitle: 'Corner Kick',
    brandUrl: 'https://github.com/UBC-Thunderbots/Software/tree/master/src/corner_kick',
});
