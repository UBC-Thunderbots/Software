/**
 * This file describes a React component that displays the heading of a settings
 * category
 */

import * as React from 'react';

export const SettingsHeading = (props: { text: string }) => {
    return <h2>{props.text}</h2>;
};
