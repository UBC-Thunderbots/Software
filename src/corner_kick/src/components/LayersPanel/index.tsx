/*
 * This file defines the UI to control the layers in the Canvas
 */

import { Button, Icon } from '@blueprintjs/core';
import { Box, Flex } from '@rebass/grid';
import * as React from 'react';

import { ILayer } from 'SRC/types';

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
    const { layers } = props;

    // If number of layers to display is 0, show a screen to indicate that there
    // is no layers.
    return (
        <>
            {layers.length > 0 ? (
                layers.map((layer) => (
                    <Flex
                        width="100%"
                        py="4px"
                        px="16px"
                        alignItems="center"
                        justifyContent="space-between"
                        key={layer.id}
                    >
                        {layer.id}
                        <Button icon="eye-open" minimal={true} />
                    </Flex>
                ))
            ) : (
                <Flex
                    width="100%"
                    height="100%"
                    flexDirection="column"
                    justifyContent="center"
                    alignItems="center"
                    my="16px"
                >
                    <Box mb="8px">
                        <Icon icon="layers" iconSize={32} />
                    </Box>
                    No layers to display
                </Flex>
            )}
        </>
    );
};
