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
                    <GridCell topStart={3} leftStart={1} leftEnd={7} middle>
                        <ControlGroup fill>
                            <Button className={Classes.FIXED} icon="flow-branch" />
                            <HTMLSelect options={['Default', 'Yellow', 'Blue']} />
                        </ControlGroup>
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
    const runAI = config['run_ai'];
    if (runAI !== undefined) {
        return (
            <Button
                icon={runAI.value ? 'pause' : 'play'}
                text={runAI.value ? 'Stop the AI' : 'Start the AI'}
                fill
                onClick={() => {
                    onClick('run_ai', !runAI.value);
                }}
            />
        );
    } else {
        return null;
    }
};

const TeamSide = ({ config, onClick }: IParamPanelProps) => {
    const override = config['override_refbox_defending_side'];
    const side = config['defending_positive_side'];

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
                            onClick('defending_positive_side', true);
                            onClick('override_refbox_defending_side', true);
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
                            onClick('defending_positive_side', false);
                            onClick('override_refbox_defending_side', true);
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
                            onClick('override_refbox_defending_side', false);
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
    const runAI = config['run_ai'];
    if (runAI !== undefined) {
        return (
            <ButtonGroup fill>
                <Tooltip
                    content="Set team color to YELLOW"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button icon={<Icon icon="dot" color="orange" />} />
                </Tooltip>
                <Tooltip
                    content="Set team color to BLUE"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button icon={<Icon icon="dot" color="blue" />} />
                </Tooltip>
                <Tooltip
                    content="Set team color to DEFAULT"
                    position={Position.BOTTOM}
                    hoverOpenDelay={500}
                >
                    <Button icon="cross" intent={Intent.PRIMARY} />
                </Tooltip>
            </ButtonGroup>
        );
    } else {
        return null;
    }
};
