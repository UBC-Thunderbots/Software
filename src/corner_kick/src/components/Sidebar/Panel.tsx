import { Icon } from '@blueprintjs/core';
import * as React from 'react';
import Scrollbars from 'react-custom-scrollbars';

import { Flex, Grid, GridCell } from 'SRC/components/Layout';
import styled from 'SRC/utils/styled-components';

const Wrapper = styled.div`
    position: absolute;
    width: 100%;
    border-bottom: 1px solid ${(props) => props.theme.colors.border};
    display: flex;
    flex-flow: column nowrap;
    overflow: hidden;
    &:last-child {
        border-bottom: none;
    }
`;

const PanelTitle = styled.div`
    font-size: 12px;
    font-weight: 700;
    text-transform: uppercase;
    color: ${(props) => props.theme.colors.subdued};
`;

interface IPanelProps {
    title: string;
    children?: React.ReactNode | React.ReactNodeArray;
}
export interface IInternalPanelProps extends IPanelProps {
    id: string;
    active?: boolean;
    onTitleClick?: () => void;
}

export const InternalPanel = (props: IInternalPanelProps) => {
    const { active, children, id, onTitleClick, title } = props;

    return (
        <Wrapper data-active={active} data-panel-id={id}>
            <Grid
                width="100%"
                height="30px"
                padding="0 8px"
                rows={1}
                columns="16px 1fr"
                onClick={onTitleClick}
                cursor="pointer"
            >
                <GridCell>
                    <Flex
                        width="100%"
                        height="100%"
                        justifyContent="center"
                        alignItems="center"
                    >
                        <Icon icon={active ? 'caret-down' : 'caret-right'} />
                    </Flex>
                </GridCell>
                <GridCell middle={true}>
                    <PanelTitle>{title}</PanelTitle>
                </GridCell>
            </Grid>
            <Scrollbars
                autoHide={true}
                style={{ flexGrow: 1, flexBasis: 0, marginBottom: '-1px' }}
            >
                {active ? children : null}
            </Scrollbars>
        </Wrapper>
    );
};

export const Panel = InternalPanel as (props: IPanelProps) => JSX.Element;
