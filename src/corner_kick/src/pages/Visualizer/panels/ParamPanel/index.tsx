import { Button, Position, Tooltip } from '@blueprintjs/core';
import { Box, Flex } from '@rebass/grid';
import * as React from 'react';

import { IROSParamState } from 'SRC/types';

interface IParamPanelProps {
    config: IROSParamState;
    onClick: (id: string, value: any) => void;
}

export class ParamPanel extends React.Component<IParamPanelProps> {
    public render() {
        const { config, onClick } = this.props;
        return (
            <Box width="100%" py="8px" px="8px">
                <Flex width="100%">
                    {config['run_ai'] && (
                        <Tooltip content="Start/Stop AI" position={Position.BOTTOM}>
                            <Button
                                icon={config['run_ai'].value ? 'pause' : 'play'}
                                minimal={true}
                                onClick={() => onClick('run_ai', !config['run_ai'].value)}
                            />
                        </Tooltip>
                    )}
                </Flex>
            </Box>
        );
    }
}
