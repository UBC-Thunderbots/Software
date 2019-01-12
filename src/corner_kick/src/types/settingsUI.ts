/**
 * This file defines the types for specifying the settings page UI
 * programatically.
 *
 * The application settings are divided into categories, which themselves contain
 * a number of settings item (individual settings entries that can be modified).
 */

import { Setting } from './settings';

/**
 * A settings category contains a number of settings entries and is
 * displayed in the UI as a subpage
 */
export interface ISettingsCategory {
    path: string;
    title: string;
    icon: string;
    settings: ISettingsItem[];
}

/**
 * A settings item is responsible for modifying a single setting entry in the
 * application
 */
export interface ISettingsItem {
    id: Setting;
    title?: string;
    description?: string;
    type: string;
}
