export interface ISettingsCategory {
    path: string;
    title: string;
    icon: string;
    settings: ISettingsItem[];
}

export interface ISettingsItem {
    id: string;
    title: string;
    description: string;
    type: string;
}

// tslint:disable:object-literal-sort-keys
export const settings: ISettingsCategory[] = [
    {
        title: 'Visualizer',
        icon: 'widgets',
        path: 'visualizer',
        settings: [
            {
                id: 'visualizer_refresh_rate',
                title: 'Visualizer refresh rate',
                description: 'The refresh rate of the visualizer',
                type: 'text',
            },
        ],
    },
    {
        title: 'ROS',
        icon: 'group_work',
        path: 'ros',
        settings: [
            {
                id: 'ros_refresh_rate',
                title: 'ROS refresh rate',
                description:
                    'The rate we check for new ROS nodes, topics, services and params.',
                type: 'text',
            },
        ],
    },
    {
        title: 'Color Theme',
        icon: 'color_lens',
        path: 'color_theme',
        settings: [
            {
                id: 'colors_fg',
                title: 'Foreground',
                description: 'Test',
                type: 'text',
            },
        ],
    },
];
