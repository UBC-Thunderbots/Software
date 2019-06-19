/***
 * This file specifies a Logo component
 */

import * as React from 'react';

import logo from 'SRC/assets/logo.svg';
import { Grid, GridCell } from 'SRC/components/Layout';
import styled from 'SRC/utils/styled-components';

const AppTitle = styled.div`
    font-size: 14px;
    font-weight: 700;
    color: ${(props) => props.theme.colors.fg};
`;

/**
 * Displays a Corner Kick logo
 */
export const Logo = () => {
    return (
        <Grid width="100%" height="100%" rows={1} columns="16px 1fr" padding="0 5px">
            <GridCell middle={true}>
                <img src={logo} width={16} height={16} />
            </GridCell>
            <GridCell middle={true}>
                <AppTitle>Corner Kick</AppTitle>
            </GridCell>
        </Grid>
    );
};
