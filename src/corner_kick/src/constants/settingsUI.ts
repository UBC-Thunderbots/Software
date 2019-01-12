/**
 * This file defines the UI of the settings page
 */

import { ISettingsCategory } from 'SRC/types';

/**
 * Defines the UI of the setting page
 */
// tslint:disable:object-literal-sort-keys
export const settingsUI: ISettingsCategory[] = [
    {
        title: 'Color Theme',
        icon: 'color_lens',
        path: 'color_theme',
        settings: [
            {
                id: 'colors_fg',
                title: 'Foreground Color',
                description: 'Color used for text and other primary UI elements',
                type: 'hex',
            },
            {
                id: 'colors_bg',
                title: 'Background Color',
                description: 'Color used for the application background',
                type: 'hex',
            },
            {
                id: 'colors_accent',
                title: 'Accent Color',
                description: 'Color used for the logo',
                type: 'hex',
            },
            {
                id: '',
                type: 'spacer',
            },
            {
                id: 'colors_selection',
                title: 'Selection Color',
                description: 'Color used for selected background',
                type: 'hex',
            },
            {
                id: 'colors_subdued',
                title: 'Subdued Color',
                description: 'Color used for subdued text',
                type: 'hex',
            },
            {
                id: 'colors_panel',
                title: 'Panel Color',
                description: 'Color used for the main panel',
                type: 'hex',
            },
            {
                id: 'colors_border',
                title: 'Border Color',
                description: 'Color used for the border between application panels',
                type: 'hex',
            },
            {
                id: '',
                type: 'spacer',
            },
            {
                id: 'colors_success',
                title: 'Success Color',
                description: 'Color used for success states',
                type: 'hex',
            },
            {
                id: 'colors_error',
                title: 'Error Color',
                description: 'Color used for error states',
                type: 'hex',
            },
            {
                id: 'colors_warn',
                title: 'Warn Color',
                description: 'Color used for warn states',
                type: 'hex',
            },
            {
                id: 'colors_info',
                title: 'Info Color',
                description: 'Color used for info states',
                type: 'hex',
            },
            {
                id: 'colors_debug',
                title: 'Debug Color',
                description: 'Color used for debug states',
                type: 'hex',
            },
            {
                id: '',
                type: 'spacer',
            },
            {
                id: 'colors_red',
                title: 'Red',
                description: 'Color used for red elements',
                type: 'hex',
            },
            {
                id: 'colors_orange',
                title: 'Orange',
                description: 'Color used for orange elements',
                type: 'hex',
            },
            {
                id: 'colors_yellow',
                title: 'Yellow',
                description: 'Color used for yellow elements',
                type: 'hex',
            },
            {
                id: 'colors_green',
                title: 'Green',
                description: 'Color used for green elements',
                type: 'hex',
            },
            {
                id: 'colors_cyan',
                title: 'Cyan',
                description: 'Color used for cyan elements',
                type: 'hex',
            },
            {
                id: 'colors_blue',
                title: 'Blue',
                description: 'Color used for blue elements',
                type: 'hex',
            },
            {
                id: 'colors_purple',
                title: 'Purple',
                description: 'Color used for purple elements',
                type: 'hex',
            },
        ],
    },
];
