import * as React from 'react';
import Scrollbars from 'react-custom-scrollbars';

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
    width: 100%;
    height: 32px;
    padding: 8px;
    display: flex;
    align-items: center;
    flex-grow: 0;
    flex-basis: 1;
    flex-shrink: 0;
    font-size: 12px;
    font-weight: 700;
    text-transform: uppercase;
    color: ${(props) => props.theme.colors.subdued};
    cursor: pointer;
    & .material-icons {
        font-size: 16px;
        margin-right: 2px;
        color: ${(props) => props.theme.colors.fg};
        transition: 0.2s all;
    }
    [data-active='false'] & .material-icons {
        transform: rotate(-90deg);
    }
    [data-disabled='true'] & {
        cursor: not-allowed;
    }
`;

interface IPanelProps {
    title: string;
    disabled?: boolean;
    children?: React.ReactNode | React.ReactNodeArray;
}
export interface IInternalPanelProps extends IPanelProps {
    id: string;
    active?: boolean;
    onTitleClick?: () => void;
}

export const InternalPanel = (props: IInternalPanelProps) => {
    const { active, children, disabled, id, onTitleClick, title } = props;

    return (
        <Wrapper data-active={active} data-disabled={disabled} data-panel-id={id}>
            <PanelTitle onClick={onTitleClick}>
                <i className="material-icons">arrow_drop_down</i>
                {title}
            </PanelTitle>
            <Scrollbars
                autoHide={true}
                style={{ flexGrow: 1, flexBasis: 0, marginBottom: '-1px' }}
            >
                {active && !disabled ? children : null}
            </Scrollbars>
        </Wrapper>
    );
};

export const Panel = InternalPanel as (props: IPanelProps) => JSX.Element;
