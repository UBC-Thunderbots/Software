/*
 * This file defines the default parameters of the application
 */

import { Colors } from '@blueprintjs/core';

import { IThemeProvider } from 'SRC/types';

// tslint:disable:object-literal-sort-keys
/**
 * Object defining the theme of the application
 */
export const theme: IThemeProvider = {
    // This is the application's color palette. Colors needs
    // to be strings as this is what our CSS code accepts.
    // Some of those attributes define color for a purpose
    // (`fg` defines the color for a foreground elements - like text)
    // while others define shades of a particular color (red defines the
    // shade of red we use)
    colors: {
        fg: Colors.GRAY1,
        bg: Colors.LIGHT_GRAY5,
        accent: Colors.FOREST5,
        panel: Colors.WHITE,
        border: Colors.LIGHT_GRAY3,
        selected: Colors.LIGHT_GRAY4,
        subdued: Colors.GRAY5,
        success: Colors.GREEN2,
        error: Colors.RED2,
        warn: Colors.ORANGE2,
        info: Colors.BLUE2,
        debug: Colors.GRAY5,
    },
};
