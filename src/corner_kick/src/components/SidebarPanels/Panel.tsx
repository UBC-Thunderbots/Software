import * as React from 'react';
import styled from 'SRC/utils/styled-components';

const Wrapper = styled.div`
    flex-grow: 1;
    width: 100%;
    border-bottom: 1px solid ${(props) => props.theme.colors.border};

    transition: 0.2s all;

    &.active {
    }

    &.inactive {
        flex-grow: 0;
    }

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

    .inactive & .material-icons {
        transform: rotate(-90deg);
    }

    .disabled & {
        cursor: not-allowed;
    }
`;

interface IPanelProps {
    title: string;
    disabled?: boolean;
}

export class Panel extends React.Component<IPanelProps> {
    public state = {
        active: true,
    };

    public render() {
        const { children, disabled, title } = this.props;
        const { active } = this.state;

        return (
            <Wrapper
                className={
                    (active && !disabled ? 'active' : 'inactive') +
                    (disabled ? ' disabled' : '')
                }
            >
                <PanelTitle onClick={this.onTitleClick}>
                    <i className="material-icons">arrow_drop_down</i>
                    {title}
                </PanelTitle>
                {active && !disabled ? children : null}
            </Wrapper>
        );
    }

    private onTitleClick = () => {
        const { disabled } = this.props;
        const { active } = this.state;
        if (!disabled) {
            this.setState({ active: !active });
        }
    };
}
