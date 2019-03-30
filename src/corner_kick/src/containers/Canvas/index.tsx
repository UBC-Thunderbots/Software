/**
 * This file defines a Canvas element. For now, this elements report how many
 * layer messages it has received.
 */

import * as React from 'react';
import { ResizeObserver } from 'resize-observer';

import { CanvasManager } from './canvasManager';

export class Canvas extends React.Component {
    private canvasManager: CanvasManager;

    private wrapperRef: React.RefObject<HTMLDivElement>;
    private resizeObserver: ResizeObserver;

    constructor(props: {}) {
        super(props);
        this.wrapperRef = React.createRef();

        this.resizeObserver = new ResizeObserver(this.handleResize);
    }

    /**
     * Initializes the shape receiver and starts requesting messages.
     */
    public componentDidMount() {
        this.canvasManager = new CanvasManager();
        this.wrapperRef.current!.appendChild(this.canvasManager.getView());

        this.resizeObserver.observe(this.wrapperRef.current!);
    }

    /**
     * Display the number of messages received.
     */
    public render() {
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
