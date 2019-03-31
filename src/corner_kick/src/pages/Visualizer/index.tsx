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
});

const mapDispatchToProps = (dispatch: Dispatch<RootAction>) =>
    bindActionCreators(
        {
            addLayer: actions.canvas.addLayer,
        },
        dispatch,
    );

interface IVisualizerProps {
    addLayer: typeof actions.canvas.addLayer;
    layers: ILayer[];
}

class VisualizerInternal extends React.Component<IVisualizerProps> {
    public render() {
        return (
            <>
                <Portal portalLocation={PortalLocation.SIDEBAR}>
                    <LayersPanel layers={this.props.layers} />
                </Portal>
                <Portal portalLocation={PortalLocation.MAIN}>
                    <Canvas layers={this.props.layers} onNewLayer={this.onNewLayer} />
                </Portal>
            </>
        );
    }

    private onNewLayer = (id: number) => {
        this.props.addLayer(id);
    };
}

export const Visualizer = connect(
    mapStateToProps,
    mapDispatchToProps,
)(VisualizerInternal);
