/**
 * This file defines a Canvas element. This provides an area for our
 * CanvasManager to operate.
 */

import * as React from 'react';
import { ResizeObserver } from 'resize-observer';

import { CanvasManager } from './canvasManager';

interface ICanvasProps {
    canvasManager: CanvasManager;
}
export class Canvas extends React.Component<ICanvasProps> {
    private wrapperRef: React.RefObject<HTMLDivElement>;
    private resizeObserver: ResizeObserver;

    constructor(props: ICanvasProps) {
        super(props);
        this.wrapperRef = React.createRef();

        this.resizeObserver = new ResizeObserver(this.handleResize);
    }

    /**
     * Attach the CanvasManager to our container
     */
    public componentDidMount() {
        this.wrapperRef.current!.appendChild(this.props.canvasManager.getView());
        this.resizeObserver.observe(this.wrapperRef.current!);
    }

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

        this.props.canvasManager.resize(width, height);
    };
}
