import { Setting } from './settings';

export interface ISettingsCategory {
    path: string;
    title: string;
    icon: string;
    settings: ISettingsItem[];
}

export interface ISettingsItem {
    id: Setting;
    title?: string;
    description?: string;
    type: string;
}
