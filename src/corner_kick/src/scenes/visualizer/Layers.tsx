import * as React from 'react';

import { ILayer } from 'SRC/types';
import styled from 'SRC/utils/styled-components';

const LayerItem = styled.div`
    width: 100%;
    height: 32px;

    padding: 0 16px;

    display: flex;
    align-items: center;

    font-size: 12px;
    color: ${(props) => props.theme.colors.subdued};

    cursor: pointer;

    & .material-icons {
        padding: 4px;
        margin-left: auto;

        border-radius: 4px;

        font-size: 14px;

        transition: 0.2s all;
    }

    & .material-icons:hover {
        background: ${(props) => props.theme.colors.bg};
    }

    &:hover {
        background: ${(props) => props.theme.colors.selected};
        color: ${(props) => props.theme.colors.fg};
    }

    &.visible .material-icons {
        color: ${(props) => props.theme.colors.fg};
    }

    &.hidden .material-icons {
        color: ${(props) => props.theme.colors.subdued};
    }
`;

interface ILayersProps {
    layers: ILayer[];
}

export const Layers = (props: ILayersProps) => {
    const { layers } = props;
    return (
        <>
            {layers.map((layer) => (
                <LayerItem
                    key={layer.topic}
                    className={layer.visible ? 'visible' : 'hidden'}
                >
                    {layer.name}
                    <i className="material-icons">remove_red_eye</i>
                </LayerItem>
            ))}
        </>
    );
};
