/***
 * This files specifies a panel used to modify ROS parameters
 */

import {
    Button,
    ButtonGroup,
    ControlGroup,
    Icon,
    HTMLSelect,
    Classes,
    Position,
    Tooltip,
    Intent,
} from '@blueprintjs/core';
import * as React from 'react';

import {
    PARAM_RUN_AI,
    PARAM_OVERRIDE_DEFENDING_SIDE,
    PARAM_DEFENDING_POSITIVE_SIDE,
    PARAM_OVERRIDE_FRIENDLY_TEAM_COLOR,
    PARAM_FRIENDLY_COLOR_YELLOW,
    PARAM_OVERRIDE_AI_PLAY,
    PARAM_CURRENT_AI_PLAY,
} from 'SRC/constants';
import { Grid, GridCell } from 'SRC/components/Layout';
import { IROSParamState } from 'SRC/types';
import styled from 'SRC/utils/styled-components';

const ParamTitle = styled.div`
    height: 100%;
    display: flex;
    align-items: flex-end;
    font-size: 11px;
    font-weight: 700;
    text-transform: uppercase;
    color: ${({ theme }) => theme.colors.subdued};
`;

interface IParamPanelProps {
    /**
     * The ROS config object, containing info about all ROS parameters
     */
    config: IROSParamState;

    /**
     * Function called when a parameter is changed
     */
    onParamChange: (id: string, value: string | boolean) => void;
}

/**
 * This panel provides the ability to change ROS parameters
 */
export class ParamPanel extends React.Component<IParamPanelProps> {
    public render() {
        return (
            <>
                <Grid width="100%" columns={6} padding="0 5px">
                    <GridCell topStart={1} leftStart={1} leftEnd={7} center>
                        <StartStopAI {...this.props} />
                    </GridCell>
                    <GridCell topStart={2} leftStart={1} leftEnd={4}>
                        <ParamTitle>Side to defend</ParamTitle>
                    </GridCell>
                    <GridCell topStart={3} leftStart={1} leftEnd={4} middle>
                        <TeamSide {...this.props} />
                    </GridCell>
                    <GridCell topStart={2} leftStart={4} leftEnd={7}>
                        <ParamTitle>Team color</ParamTitle>
                    </GridCell>
                    <GridCell topStart={3} leftStart={4} leftEnd={7} middle>
                        <TeamColor {...this.props} />
                    </GridCell>
                    <GridCell topStart={4} leftStart={1} leftEnd={7}>
                        <ParamTitle>AI Play Type</ParamTitle>
                    </GridCell>
                    <GridCell topStart={5} leftStart={1} leftEnd={7} middle>
                        <AIPlay {...this.props} />
                    </GridCell>
                </Grid>
            </>
        );
    }
}

/**
 * UI element to start/stop the Thunderbots AI
 */
const StartStopAI = ({ config, onParamChange }: IParamPanelProps) => {
    const runAI = config[PARAM_RUN_AI];
    if (runAI !== undefined) {
        return (
            <Button
                icon={runAI.value ? 'pause' : 'play'}
                text={runAI.value ? 'Stop the AI' : 'Start the AI'}
                fill
                onClick={() => {
                    onParamChange(PARAM_RUN_AI, !runAI.value);
                }}
            />
        );
    } else {
        return null;
    }
};

/**
 * UI element to select the robot defense side
 */
const TeamSide = ({ config, onParamChange }: IParamPanelProps) => {
    const override = config[PARAM_OVERRIDE_DEFENDING_SIDE];
    const side = config[PARAM_DEFENDING_POSITIVE_SIDE];

    if (override !== undefined) {
        return (
            <ButtonGroup fill>
                <Tooltip
                    content="Set defending side to EAST"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button
                        icon="direction-left"
                        intent={
                            override.value && side.value ? Intent.PRIMARY : Intent.NONE
                        }
                        onClick={() => {
                            onParamChange(PARAM_DEFENDING_POSITIVE_SIDE, true);
                            onParamChange(PARAM_OVERRIDE_DEFENDING_SIDE, true);
                        }}
                    />
                </Tooltip>
                <Tooltip
                    content="Set defending side to WEST"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button
                        icon="direction-right"
                        intent={
                            override.value && !side.value ? Intent.PRIMARY : Intent.NONE
                        }
                        onClick={() => {
                            onParamChange(PARAM_DEFENDING_POSITIVE_SIDE, false);
                            onParamChange(PARAM_OVERRIDE_DEFENDING_SIDE, true);
                        }}
                    />
                </Tooltip>
                <Tooltip
                    content="Set defending side to DEFAULT"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button
                        icon="cross"
                        intent={!override.value ? Intent.PRIMARY : Intent.NONE}
                        onClick={() => {
                            onParamChange(PARAM_OVERRIDE_DEFENDING_SIDE, false);
                        }}
                    />
                </Tooltip>
            </ButtonGroup>
        );
    } else {
        return null;
    }
};

/**
 * UI element to specify the robot team color
 */
const TeamColor = ({ config, onParamChange }: IParamPanelProps) => {
    const override = config[PARAM_OVERRIDE_FRIENDLY_TEAM_COLOR];
    const color = config[PARAM_FRIENDLY_COLOR_YELLOW];

    if (override !== undefined) {
        return (
            <ButtonGroup fill>
                <Tooltip
                    content="Set team color to YELLOW"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button
                        icon={<Icon icon="dot" color="orange" />}
                        intent={
                            override.value && color.value ? Intent.PRIMARY : Intent.NONE
                        }
                        onClick={() => {
                            onParamChange(PARAM_FRIENDLY_COLOR_YELLOW, true);
                            onParamChange(PARAM_OVERRIDE_FRIENDLY_TEAM_COLOR, true);
                        }}
                    />
                </Tooltip>
                <Tooltip
                    content="Set team color to BLUE"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button
                        icon={<Icon icon="dot" color="cyan" />}
                        intent={
                            override.value && !color.value ? Intent.PRIMARY : Intent.NONE
                        }
                        onClick={() => {
                            onParamChange(PARAM_FRIENDLY_COLOR_YELLOW, false);
                            onParamChange(PARAM_OVERRIDE_FRIENDLY_TEAM_COLOR, true);
                        }}
                    />
                </Tooltip>
                <Tooltip
                    content="Set team color to DEFAULT"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button
                        icon="cross"
                        intent={!override.value ? Intent.PRIMARY : Intent.NONE}
                        onClick={() => {
                            onParamChange(PARAM_OVERRIDE_FRIENDLY_TEAM_COLOR, false);
                        }}
                    />
                </Tooltip>
            </ButtonGroup>
        );
    } else {
        return null;
    }
};

/**
 * UI element to specify the current AI play type
 */
const AIPlay = ({ config, onParamChange }: IParamPanelProps) => {
    const override = config[PARAM_OVERRIDE_AI_PLAY];
    const play = config[PARAM_CURRENT_AI_PLAY];

    if (override !== undefined && play.options !== undefined) {
        return (
            <ControlGroup fill>
                <Tooltip
                    className={Classes.FIXED}
                    content="Set current AI play"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button icon="layout-sorted-clusters" />
                </Tooltip>
                <HTMLSelect
                    options={['Do not override', ...play.options]}
                    value={!override.value ? 'Do not override' : play.value}
                    onChange={(e) => {
                        const value = e.target.value;
                        if (value === 'Do not override') {
                            onParamChange(PARAM_OVERRIDE_AI_PLAY, false);
                        } else {
                            onParamChange(PARAM_OVERRIDE_AI_PLAY, true);
                            onParamChange(PARAM_CURRENT_AI_PLAY, value);
                        }
                    }}
                />
            </ControlGroup>
        );
    } else {
        return null;
    }
};
