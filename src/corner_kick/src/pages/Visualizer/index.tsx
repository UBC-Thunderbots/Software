import * as React from 'react';
import { connect } from 'react-redux';
import { bindActionCreators, Dispatch } from 'redux';

import { LayersPanel } from 'SRC/components/LayersPanel';
import { Portal, PortalLocation } from 'SRC/components/Portal';
import { Canvas } from 'SRC/containers/Canvas';
import { actions, RootAction } from 'SRC/store';
import { ILayer, IRootState } from 'SRC/types';

const mapStateToProps = (state: IRootState) => ({
    layers: state.canvas.layers,
    layerOrder: state.canvas.layerOrder,
});

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

    private onNewLayer = (id: number) => {
        this.props.addLayer(id);
    };

    private onLayerVisibilityToggle = (id: number) => {
        this.props.toggleVisibility(id);
    };
}

export const Visualizer = connect(
    mapStateToProps,
    mapDispatchToProps,
)(VisualizerInternal);
