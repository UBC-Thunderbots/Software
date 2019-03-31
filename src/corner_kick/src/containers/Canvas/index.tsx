/**
 * This file defines a Canvas element. For now, this elements report how many
 * layer messages it has received.
 */

import * as React from 'react';
import { ResizeObserver } from 'resize-observer';

import { ILayer } from 'SRC/types';
import { CanvasManager } from './canvasManager';

interface ICanvasProps {
    layers: ILayer[];
    onNewLayer: (id: number) => void;
}
export class Canvas extends React.Component<ICanvasProps> {
    private canvasManager: CanvasManager;

    private wrapperRef: React.RefObject<HTMLDivElement>;
    private resizeObserver: ResizeObserver;

    constructor(props: ICanvasProps) {
        super(props);
        this.wrapperRef = React.createRef();

        this.canvasManager = new CanvasManager(this.props.onNewLayer);
        this.resizeObserver = new ResizeObserver(this.handleResize);
    }

    /**
     * Initializes the shape receiver and starts requesting messages.
     */
    public componentDidMount() {
        this.wrapperRef.current!.appendChild(this.canvasManager.getView());

        this.resizeObserver.observe(this.wrapperRef.current!);
    }

    /**
     * Display the number of messages received.
     */
    public render() {
        this.canvasManager.handleLayerOperations(this.props.layers);
        return <div ref={this.wrapperRef} style={{ width: '100%', height: '100%' }} />;
    }

    /**
     * Clean-up after ourselves to avoid memory leaks
     */
    public componentWillUnmount() {
        this.resizeObserver.disconnect();
    }

    private handleResize = () => {
        const width = this.wrapperRef.current!.clientWidth;
        const height = this.wrapperRef.current!.clientHeight;

        this.canvasManager.resize(width, height);
    };
}
