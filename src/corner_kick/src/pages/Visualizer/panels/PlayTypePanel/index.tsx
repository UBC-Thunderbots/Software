/***
 * This file defines the UI to view the current tactic
 */

import { Box } from '@rebass/grid';
import * as React from 'react';

interface IPlayTypeProps {
    playType: string;
    playName: string;
    tactics: string[];
}

/**
 * Describes a panel showing the current tactics in the AI.
 */
export const PlayTypePanel = (props: IPlayTypeProps) => {
    const { playType, playName, tactics } = props;

    return (
        <Box width="100%" py="8px" px="8px">
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
