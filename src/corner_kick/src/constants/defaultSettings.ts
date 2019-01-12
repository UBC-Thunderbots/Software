/**
 * This file defines the default parameters of the application
 */

import { ISettingsState } from 'SRC/types';

// tslint:disable:object-literal-sort-keys
/**
 * Object defining all the default parameters of the application
 */
export const defaultSettings: ISettingsState = {
    ['']: '',
    // Default settings for the color scheme
    colors_fg: '#B3B1AD',
    colors_bg: '#0A0E14',
    colors_accent: '#E6B450',
    colors_panel: '#0D1016',
    colors_border: '#01060E',
    colors_selection: '#161F2A',
    colors_subdued: '#3D424D',
    colors_success: '#C2D94C',
    colors_error: '#FF3333',
    colors_warn: '#FF8F40',
    colors_info: '#39BAE6',
    colors_debug: '#3D424D',
    colors_red: '#FF3333',
    colors_orange: '#FF8F40',
    colors_yellow: '#FFEE99',
    colors_green: '#C2D94C',
    colors_cyan: '#39BAE6',
    colors_blue: '#6994BF',
    colors_purple: '#D4BFFF',
};
