/**
 * This file specifies the theme of Storybook
 */

import { create } from '@storybook/theming';

import logo from 'SRC/assets/logo.svg';

import { theme } from 'SRC/constants';

export default create({
    base: 'dark',

    brandImage: logo,
    brandTitle: 'Corner Kick',
    brandUrl: 'https://github.com/UBC-Thunderbots/Software/tree/master/src/corner_kick',

    colorPrimary: theme.colors.green,
    colorSecondary: theme.colors.accent,

    // UI
    appBg: theme.colors.bg,
    appBorderColor: theme.colors.border,
    appBorderRadius: 4,
    appContentBg: theme.colors.panel,

    // Typography
    fontBase: '"Open Sans", sans-serif',
    fontCode: 'monospace',

    // Text colors
    textColor: theme.colors.fg,
    textInverseColor: 'black',

    // Toolbar default and active colors
    barBg: theme.colors.panel,
    barSelectedColor: theme.colors.fg,
    barTextColor: theme.colors.subdued,

    // Form colors
    inputBg: theme.colors.bg,
    inputBorder: theme.colors.border,
    inputBorderRadius: 4,
    inputTextColor: theme.colors.fg,
});
