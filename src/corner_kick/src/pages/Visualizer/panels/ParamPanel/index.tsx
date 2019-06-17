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
} from 'SRC/constants';
import { Grid, GridCell } from 'SRC/components/Layout';
import { IROSParamState } from 'SRC/types';

interface IParamPanelProps {
    config: IROSParamState;
    onClick: (id: string, value: any) => void;
}

export class ParamPanel extends React.Component<IParamPanelProps> {
    public render() {
        return (
            <>
                <Grid width="100%" columns={6} padding="0 5px">
                    <GridCell topStart={1} leftStart={1} leftEnd={7} center>
                        <StartStopAI {...this.props} />
                    </GridCell>
                    <GridCell topStart={2} leftStart={1} leftEnd={4} middle>
                        <TeamSide {...this.props} />
                    </GridCell>
                    <GridCell topStart={2} leftStart={4} leftEnd={7} middle>
                        <TeamColor {...this.props} />
                    </GridCell>
                    <GridCell topStart={4} leftStart={1} leftEnd={7} middle>
                        <ControlGroup fill>
                            <Button
                                className={Classes.FIXED}
                                icon="layout-sorted-clusters"
                            />
                            <HTMLSelect options={['Default', 'Yellow', 'Blue']} />
                        </ControlGroup>
                    </GridCell>
                </Grid>
            </>
        );
    }
}

const StartStopAI = ({ config, onClick }: IParamPanelProps) => {
    const runAI = config[PARAM_RUN_AI];
    if (runAI !== undefined) {
        return (
            <Button
                icon={runAI.value ? 'pause' : 'play'}
                text={runAI.value ? 'Stop the AI' : 'Start the AI'}
                fill
                onClick={() => {
                    onClick(PARAM_RUN_AI, !runAI.value);
                }}
            />
        );
    } else {
        return null;
    }
};

const TeamSide = ({ config, onClick }: IParamPanelProps) => {
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
                            onClick(PARAM_DEFENDING_POSITIVE_SIDE, true);
                            onClick(PARAM_OVERRIDE_DEFENDING_SIDE, true);
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
                            onClick(PARAM_DEFENDING_POSITIVE_SIDE, false);
                            onClick(PARAM_OVERRIDE_DEFENDING_SIDE, true);
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
                            onClick(PARAM_OVERRIDE_DEFENDING_SIDE, false);
                        }}
                    />
                </Tooltip>
            </ButtonGroup>
        );
    } else {
        return null;
    }
};

const TeamColor = ({ config, onClick }: IParamPanelProps) => {
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
                            onClick(PARAM_FRIENDLY_COLOR_YELLOW, true);
                            onClick(PARAM_OVERRIDE_FRIENDLY_TEAM_COLOR, true);
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
                            onClick(PARAM_FRIENDLY_COLOR_YELLOW, false);
                            onClick(PARAM_OVERRIDE_FRIENDLY_TEAM_COLOR, true);
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
                            onClick(PARAM_OVERRIDE_FRIENDLY_TEAM_COLOR, false);
                        }}
                    />
                </Tooltip>
            </ButtonGroup>
        );
    } else {
        return null;
    }
};
