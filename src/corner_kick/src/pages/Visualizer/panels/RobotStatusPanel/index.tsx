/***
 * This file defines the UI to view robot statuses
 */

import { Tag } from '@blueprintjs/core';
import { Box } from '@rebass/grid';
import * as React from 'react';

import { Flex } from 'SRC/components/Layout';
import { IRobotStatuses } from 'SRC/types';
import { theme } from 'SRC/constants';

interface IRobotStatusPanelProps {
    robotStatuses: IRobotStatuses;
}

/**
 * Displays robot statuses sorted by time
 */
export const RobotStatusPanel = (props: IRobotStatusPanelProps) => {
    // Sort by time, then by robot number, then by message alphabetically
    const sortedRobotStatuses = Object.values(props.robotStatuses).sort((a, b) => {
        const timeDelta = a.timestamp - b.timestamp;
        const robotDelta = a.robot - b.robot;
        if (timeDelta != 0) {
            return timeDelta;
        } else if (robotDelta != 0) {
            return robotDelta;
        } else {
            return a.message.localeCompare(b.message);
        }
    });

    return (
        <Box padding="2px 0">
            {sortedRobotStatuses.map((status) => (
                <Flex
                    key={`${status.robot}:${status.message}`}
                    width="100%"
                    alignItems="center"
                    padding="2px 5px"
                >
                    <Tag
                        style={{
                            background:
                                theme.qualitativeColorScale[
                                    status.robot % theme.qualitativeColorScale.length
                                ],
                        }}
                    >
                        Robot {status.robot}
                    </Tag>
                    <div style={{ marginLeft: '5px' }}>{status.message}</div>
                    <Tag style={{ marginLeft: 'auto' }}>{status.timestamp} s. ago</Tag>
                </Flex>
            ))}
        </Box>
    );
};
