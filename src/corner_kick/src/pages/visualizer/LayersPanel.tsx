/*
 * This file defines the UI to control the layers of the visualizer
 */

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

    &:hover {
        background: ${(props) => props.theme.colors.selected};
        color: ${(props) => props.theme.colors.fg};
    }
`;

const LayerVisibilityToggle = styled('div')<{ visible: boolean }>`
    padding: 4px;
    margin-left: auto;

    border-radius: 4px;

    color: ${(props) =>
        props.visible ? props.theme.colors.fg : props.theme.colors.subdued};

    font-size: 14px;

    transition: 0.2s all;

    &:hover {
        background: ${(props) => props.theme.colors.bg};
    }
`;

interface ILayersProps {
    layers: ILayer[];
}

export const LayersPanel = (props: ILayersProps) => {
    const { layers } = props;
    return (
        <>
            {layers.map((layer) => (
                <LayerItem key={layer.id}>
                    {layer.name}
                    <LayerVisibilityToggle
                        visible={layer.visible}
                        className="material-icons"
                    >
                        remove_red_eye
                    </LayerVisibilityToggle>
                </LayerItem>
            ))}
        </>
    );
};
