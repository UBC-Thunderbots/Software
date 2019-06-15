import { Button, Popover, Position, Menu, MenuItem } from '@blueprintjs/core';
import * as React from 'react';

import logo from 'SRC/assets/logo.svg';
import { Grid, GridCell } from 'SRC/components/Layout';
import styled from 'SRC/utils/styled-components';

const AppTitle = styled.div`
    font-size: 14px;
    font-weight: 700;
    color: ${(props) => props.theme.colors.fg};
`;

export const Logo = () => {
    return (
        <Grid width="100%" height="100%" rows={1} columns="16px 1fr 30px" padding="0 8px">
            <GridCell middle={true}>
                <img src={logo} width={16} height={16} />
            </GridCell>
            <GridCell middle={true}>
                <AppTitle>Corner Kick</AppTitle>
            </GridCell>
            <GridCell middle={true}>
                <Popover
                    content={
                        <Menu>
                            <MenuItem
                                text="Make application fullscreen"
                                onClick={() => document.body.requestFullscreen()}
                            />
                            <MenuItem
                                text="Make main view fullscreen"
                                onClick={() =>
                                    document.getElementById('main')!.requestFullscreen()
                                }
                            />
                            <MenuItem
                                text="Make console fullscreen"
                                onClick={() =>
                                    document
                                        .getElementById('console')!
                                        .requestFullscreen()
                                }
                            />
                        </Menu>
                    }
                    position={Position.TOP_RIGHT}
                >
                    <Button icon="menu" minimal={true} />
                </Popover>
            </GridCell>
        </Grid>
    );
};
