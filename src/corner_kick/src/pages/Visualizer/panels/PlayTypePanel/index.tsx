/*
 * This file defines the UI to control the layers in the Canvas
 */

import { Box } from '@rebass/grid';
import * as React from 'react';
import { theme } from 'SRC/constants';

interface IPlayTypeProps {
    playType: string;
    playName: string;
    tactics: string[];
}

/**
 * Describes a panel showing the active layers inside the application.
 *
 * Supports an empty state
 */
export const PlayTypePanel = (props: IPlayTypeProps) => {
    const { playType, playName, tactics } = props;

    // If number of layers to display is 0, show a screen to indicate that there
    // is no layers.
    return (
        <Box
            width="100%"
            py="16px"
            px="16px"
            style={{
                borderTop: `1px solid ${theme.colors.border}`,
                borderBottom: `1px solid ${theme.colors.border}`,
            }}
        >
            <b>Play Type: </b>
            {playType}
            <br />
            <b>Play Name: </b>
            {playName}
            <br />
            <b>Tactics:</b>
            <br />
            {tactics.map((tactic) => (
                <>
                    {tactic}
                    <br />
                </>
            ))}
        </Box>
    );
};
