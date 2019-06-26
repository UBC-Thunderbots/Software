/***
 * This file specifies the Visualizer page
 *
 * This page contains UI for the Canvas
 */

import * as React from 'react';
import { connect } from 'react-redux';
import { bindActionCreators, Dispatch } from 'redux';

import { Sidebar, Panel } from 'SRC/components/Sidebar';
import { Portal, PortalLocation } from 'SRC/components/Portal';
import { SPRITESHEET } from 'SRC/constants';
import { Canvas, CanvasManager, LayerReceiver } from 'SRC/containers/Canvas';
import { actions, RootAction } from 'SRC/store';
import { ILayer, IRootState, IROSParamState, IRobotStatuses } from 'SRC/types';

import { ParamPanel } from './panels/ParamPanel';
import { LayersPanel } from './panels/LayersPanel';
import { PlayTypePanel } from './panels/PlayTypePanel';
import { RobotStatusPanel } from './panels/RobotStatusPanel/index';

// We request the layer data from the store
const mapStateToProps = (state: IRootState) => ({
    layers: state.canvas.layers,
    layerOrder: state.canvas.layerOrder,
    playType: state.thunderbots.playType,
    playName: state.thunderbots.playName,
    tactics: state.thunderbots.tactics,
    params: state.rosParameters,
    robotStatuses: state.robotStatus.statuses,
});

// We request layer related actions
const mapDispatchToProps = (dispatch: Dispatch<RootAction>) =>
    bindActionCreators(
        {
            addLayer: actions.canvas.addLayer,
            toggleVisibility: actions.canvas.toggleLayerVisibility,
            setParam: actions.rosParameters.setParam,
        },
        dispatch,
    );

interface IVisualizerProps {
    layers: { [id: number]: ILayer };
    layerOrder: number[];
    playType: string;
    playName: string;
    tactics: string[];
    params: IROSParamState;
    robotStatuses: IRobotStatuses;

    // Actions
    addLayer: typeof actions.canvas.addLayer;
    toggleVisibility: typeof actions.canvas.toggleLayerVisibility;
    setParam: typeof actions.rosParameters.setParam;
}

class VisualizerInternal extends React.Component<IVisualizerProps> {
    public canvasManager: CanvasManager;
    public layerReceiver: LayerReceiver;

    /**
     * Initialize the Canvas Manager and Layer Receiver
     */
    constructor(props: IVisualizerProps) {
        super(props);
        this.canvasManager = new CanvasManager(SPRITESHEET);
        this.layerReceiver = new LayerReceiver(this.onNewLayerData);
    }

    public componentDidMount() {
        this.layerReceiver.connect();
    }

    public render() {
        // We use the layer order information in the store to create an ordered
        // array of the layers to be consumed by our UI.
        const orderedLayers = this.props.layerOrder.map((key) => this.props.layers[key]);

        // Update layer ordering and visibility
        this.canvasManager.handleLayerOperations(orderedLayers);

        return (
            <>
                <Portal portalLocation={PortalLocation.SIDEBAR}>
                    <Sidebar inactivePanelHeight={32} minPanelHeight={200}>
                        <Panel title="Layers">
                            <LayersPanel
                                layers={orderedLayers}
                                toggleVisibility={this.onLayerVisibilityToggle}
                            />
                        </Panel>
                        <Panel title="AI Controls">
                            <ParamPanel
                                config={this.props.params}
                                onParamChange={this.props.setParam}
                            />
                        </Panel>
                        <Panel title="AI Status">
                            <PlayTypePanel {...this.props} />
                        </Panel>
                    </Sidebar>
                </Portal>
                <Portal portalLocation={PortalLocation.MAIN}>
                    <Canvas canvasManager={this.canvasManager} />
                </Portal>
                <Portal portalLocation={PortalLocation.CONSOLE}>
                    <RobotStatusPanel robotStatuses={this.props.robotStatuses} />
                </Portal>
            </>
        );
    }

    public componentWillUnmount() {
        this.layerReceiver.close();
    }

    /**
     * Called when we received layer data from the websocket
     */
    private onNewLayerData = (data: ArrayBuffer) => {
        this.canvasManager.handleLayerMessage(data, this.onNewLayer);
    };

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
