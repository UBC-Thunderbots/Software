/***
 * This file defines the UI to view the current tactic
 */

import { Button } from '@blueprintjs/core';
import { Box } from '@rebass/grid';
import * as React from 'react';
import * as ROS from 'SRC/utils/ros';

/**
 * Describes a panel showing the current tactics in the AI.
 */
export const SettingsPanel = () => {
    function startRunAi() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'run_ai', value: true }],
                },
            },
        );
    }

    function stopRunAi() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'run_ai', value: false }],
                },
            },
        );
    }

    function enableOverrideRefboxFriendlyTeamColor() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'override_refbox_friendly_team_color', value: true }],
                },
            },
        );
    }

    function disableOverrideRefboxFriendlyTeamColor() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [
                        { name: 'override_refbox_friendly_team_color', value: false },
                    ],
                },
            },
        );
    }

    function enableFriendlyColorYellow() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'friendly_color_yellow', value: true }],
                },
            },
        );
    }

    function disableFriendlyColorYellow() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'friendly_color_yellow', value: false }],
                },
            },
        );
    }

    function enableOverrideRefboxDefendingSide() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'override_refbox_defending_side', value: true }],
                },
            },
        );
    }

    function disableOverrideRefboxDefendingSide() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'override_refbox_defending_side', value: false }],
                },
            },
        );
    }

    function enableDefendingPositiveSide() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'defending_positive_side', value: true }],
                },
            },
        );
    }

    function disableDefendingPositiveSide() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'defending_positive_side', value: false }],
                },
            },
        );
    }

    function enableOverrideRefboxPlay() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'override_refbox_play', value: true }],
                },
            },
        );
    }

    function disableOverrideRefboxPlay() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'override_refbox_play', value: false }],
                },
            },
        );
    }

    function enableOverrideAiPlay() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'override_ai_play', value: true }],
                },
            },
        );
    }

    function disableOverrideAiPlay() {
        ROS.sendRequestToService(
            '/ai_control/set_parameters',
            'dynamic_reconfigure/Reconfigure',
            {
                config: {
                    bools: [{ name: 'override_ai_play', value: false }],
                },
            },
        );
    }

    return (
        <Box>
            <div>
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Start run_ai"
                        onClick={startRunAi}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Stop run_ai"
                        onClick={stopRunAi}
                    />
                </div>
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Enable override_refbox_friendly_team_color"
                        onClick={enableOverrideRefboxFriendlyTeamColor}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Disable override_refbox_friendly_team_color"
                        onClick={disableOverrideRefboxFriendlyTeamColor}
                    />
                </div>
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Enable friendly_color_yellow"
                        onClick={enableFriendlyColorYellow}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Disable friendly_color_yellow"
                        onClick={disableFriendlyColorYellow}
                    />
                </div>
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Enable Override_refbox_defending_side"
                        onClick={enableOverrideRefboxDefendingSide}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Disable Override_refbox_defending_side"
                        onClick={disableOverrideRefboxDefendingSide}
                    />
                </div>
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Enable friendly_color_yellow"
                        onClick={enableDefendingPositiveSide}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Disable friendly_color_yellow"
                        onClick={disableDefendingPositiveSide}
                    />
                </div>
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Enable override_refbox_defending_side"
                        onClick={enableOverrideRefboxPlay}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Disable override_refbox_defending_side"
                        onClick={disableOverrideRefboxPlay}
                    />
                </div>{' '}
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Enable defending_positive_side"
                        onClick={enableDefendingPositiveSide}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Disable defending_positive_side"
                        onClick={disableDefendingPositiveSide}
                    />
                </div>
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Enable override_refbox_play"
                        onClick={enableOverrideRefboxPlay}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Disable override_refbox_play"
                        onClick={disableOverrideRefboxPlay}
                    />
                </div>
                <div>
                    <Button
                        intent="success"
                        minimal={true}
                        text="Enable override_ai_play"
                        onClick={enableOverrideAiPlay}
                    />
                    <Button
                        intent="danger"
                        minimal={true}
                        text="Disable override_ai_play"
                        onClick={disableOverrideAiPlay}
                    />
                </div>
            </div>
        </Box>
    );
};
