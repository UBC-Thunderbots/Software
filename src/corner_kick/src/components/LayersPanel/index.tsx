/*
 * This file defines the UI to control the layers in the Canvas
 */

import * as React from 'react';

import { ILayer } from 'SRC/types';
import styled from 'SRC/utils/styled-components';

/**
 * Styling when the panel is does not have any layers
 */
const EmptyWrapper = styled.div`
    width: 100%;
    height: 100px;

    display: flex;
    flex-flow: column nowrap;
    justify-content: center;
    align-items: center;

    color: ${(props) => props.theme.colors.subdued};
`;

/**
 * Styling for the icon describing the empty layer state
 */
const EmptyWrapperIcon = styled.i`
    font-size: 24px;
    margin: 4px;
`;

/**
 * Styling for a layer item
 */
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

/**
 * Styling for the visibility toggle inside each layer item
 */
const LayerVisibilityToggle = styled('i')<{ visible: boolean }>`
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
    /**
     * The layers to display
     */
    layers: ILayer[];

    /**
     * Callback that gets triggered when the visibility is toggled
     * on a layer
     */
    toggleVisibility: (id: number) => void;
}

/**
 * Describes a panel showing the active layers inside the application.
 *
 * Supports an empty state
 */
export const LayersPanel = (props: ILayersProps) => {
    const { layers, toggleVisibility } = props;

    // If number of layers to display is 0, show a screen to indicate that there
    // is no layers.
    return (
        <>
            {layers.length > 0 ? (
                layers.map((layer) => (
                    <LayerItem key={layer.id}>
                        {layer.id}
                        <LayerVisibilityToggle
                            visible={layer.visible}
                            className="material-icons"
                            onClick={(e) => {
                                toggleVisibility(layer.id);
                            }}
                        >
                            remove_red_eye
                        </LayerVisibilityToggle>
                    </LayerItem>
                ))
            ) : (
                <EmptyWrapper>
                    <EmptyWrapperIcon className="material-icons">
                        layers_clear
                    </EmptyWrapperIcon>
                    No layers to display
                </EmptyWrapper>
            )}
        </>
    );
};
