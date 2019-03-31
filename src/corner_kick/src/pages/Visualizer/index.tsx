/***
 * This file specifies the Visualizer page
 *
 * This page contains UI for the Canvas
 */

import * as React from 'react';
import { connect } from 'react-redux';
import { bindActionCreators, Dispatch } from 'redux';

import { LayersPanel } from 'SRC/components/LayersPanel';
import { Portal, PortalLocation } from 'SRC/components/Portal';
import { Canvas } from 'SRC/containers/Canvas';
import { actions, RootAction } from 'SRC/store';
import { ILayer, IRootState } from 'SRC/types';

// We request the layer data from the store
const mapStateToProps = (state: IRootState) => ({
    layers: state.canvas.layers,
    layerOrder: state.canvas.layerOrder,
});

// We request layer related actions
const mapDispatchToProps = (dispatch: Dispatch<RootAction>) =>
    bindActionCreators(
        {
            addLayer: actions.canvas.addLayer,
            toggleVisibility: actions.canvas.toggleLayerVisibility,
        },
        dispatch,
    );

interface IVisualizerProps {
    addLayer: typeof actions.canvas.addLayer;
    toggleVisibility: typeof actions.canvas.toggleLayerVisibility;
    layers: { [id: number]: ILayer };
    layerOrder: number[];
}

class VisualizerInternal extends React.Component<IVisualizerProps> {
    public render() {
        // We use the layer order information in the store to create an ordered
        // array of the layers to be consumed by our UI.
        const orderedLayers = this.props.layerOrder.map((key) => this.props.layers[key]);
        return (
            <>
                <Portal portalLocation={PortalLocation.SIDEBAR}>
                    <LayersPanel
                        layers={orderedLayers}
                        toggleVisibility={this.onLayerVisibilityToggle}
                    />
                </Portal>
                <Portal portalLocation={PortalLocation.MAIN}>
                    <Canvas layers={orderedLayers} onNewLayer={this.onNewLayer} />
                </Portal>
            </>
        );
    }

    /**
     * Called when a new layer is received by the Canvas. We dispatch an action
     * to update the rest of our UI accordingly.
     */
    private onNewLayer = (id: number) => {
        this.props.addLayer(id);
    };

    /**
     * Called when the visibility is toggled on a particular layer. We dispatch an action
     * to update the rest of our UI accordingly.
     */
    private onLayerVisibilityToggle = (id: number) => {
        this.props.toggleVisibility(id);
    };
}

export const Visualizer = connect(
    mapStateToProps,
    mapDispatchToProps,
)(VisualizerInternal);
