/*
 * This file defines the default parameters of the application
 */

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
        fg: '#B3B1AD',
        bg: '#0A0E14',
        accent: '#E6B450',
        panel: '#0D1016',
        border: '#01060E',
        selected: '#161F2A',
        subdued: '#3D424D',
        success: '#C2D94C',
        error: '#FF3333',
        warn: '#FF8F40',
        info: '#39BAE6',
        debug: '#3D424D',
        red: '#FF3333',
        orange: '#FF8F40',
        yellow: '#FFEE99',
        green: '#C2D94C',
        cyan: '#39BAE6',
        blue: '#6994BF',
        purple: '#D4BFFF',
    },
};
