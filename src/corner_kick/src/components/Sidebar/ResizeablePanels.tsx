/***
 * This file specifies a React component enabling resizeable panels.
 * The actual resizing logic is specified in utils.ts in the same directory
 */

import * as React from 'react';
import { ResizeObserver } from 'resize-observer';

import { isChrome } from 'SRC/utils/browserDetection';
import styled from 'SRC/utils/styled-components';

import { IInternalPanelProps, Panel } from './Panel';
import {
    getInitialSize,
    IPanel,
    makePanelActive,
    makePanelInactive,
    resize,
    resizeParent,
} from './utils';

/**
 * Styling for the parent container
 */
const Wrapper = styled.div`
    position: relative;
    width: 100%;
    height: 100%;

    overflow: hidden;
`;

/**
 * Styling for the resizer
 */
const Resizer = styled.div`
    position: absolute;
    width: 100%;
    height: 10px;

    cursor: row-resize;
`;

export interface IResizeablePanelsProps {
    /**
     * The minimum height of the panel
     */
    minPanelHeight: number;

    /**
     * The height of an inactive panel
     */
    inactivePanelHeight: number;

    /**
     * The sidebar only supports Panel components as its children
     */
    children: React.ReactElement<typeof Panel> | Array<React.ReactElement<typeof Panel>>;
}

/**
 * Implements a resizeable panel container, allowing for the resizing of
 * panels
 *
 * Some of the logic for resizing is defined in Panel
 */
export class ResizeablePanels extends React.Component<IResizeablePanelsProps> {
    /**
     *  Reference to the DOM node of the parent container
     * Used to determine the height of the parent container
     */
    private wrapperRef: React.RefObject<HTMLDivElement>;

    /**
     * Reference to the style node in `<head>`
     * Used to change the height of panels in a performant way
     */
    private styleElement: HTMLStyleElement;

    /**
     * Index of the resizer element currently being
     * resized.
     *
     * Value is -1 if resize is not in progress
     */
    private resizeIndex = -1;

    /**
     * Counter measuring the distance between the resizer and the mouse position
     * Used to perform the resize operation
     */
    private mouseDelta = 0;

    /**
     * Current height of the parent container
     */
    private parentHeight = 0;

    /**
     * Information regarding the panel in the container
     */
    private panels: IPanel[];

    /**
     * Current device pixel ratio
     */
    private pixelRatio: number;

    /**
     * Allows to be notified when the parent container's height is changed
     */
    private resizeObserver: ResizeObserver;

    constructor(props: IResizeablePanelsProps) {
        super(props);

        // Fetch the reference of the parent container
        this.wrapperRef = React.createRef();

        // Generate initial information for the panels
        this.panels = getInitialSize(React.Children.count(this.props.children), 100);
    }

    public componentDidMount() {
        // Start observing if the parent container has changed height
        this.resizeObserver = new ResizeObserver(this.onResizeParent);
        this.resizeObserver.observe(this.wrapperRef.current!);

        // We need information regarding device pixel ratio
        this.pixelRatio = window.devicePixelRatio;

        // Create a style element to allow for a performant panel height change
        this.styleElement = document.createElement('style');
        this.styleElement.type = 'text/css';

        const head = document.querySelector('head');
        head!.appendChild(this.styleElement);

        // Update panels so they take the parent's height
        this.parentHeight = this.wrapperRef.current!.clientHeight;
        this.panels = getInitialSize(
            React.Children.count(this.props.children),
            this.parentHeight,
        );

        // Update panel heights
        this.setSizes();
    }

    public componentWillUnmount() {
        // Always clean up after yourself
        // Remove the listener of the parent container's height
        this.resizeObserver.unobserve(this.wrapperRef.current!);
    }

    public render() {
        const { children, minPanelHeight } = this.props;

        const count = React.Children.count(children);

        return (
            <Wrapper ref={this.wrapperRef} style={{ minHeight: count * minPanelHeight }}>
                {React.Children.map(
                    children,
                    (child: React.ReactElement<IInternalPanelProps>, index) => {
                        return (
                            <>
                                {React.cloneElement(child, {
                                    active: this.panels[index].active,
                                    id: `${index}`,
                                    onTitleClick: this.onTitleClick(index),
                                })}
                                {index !== count - 1 ? (
                                    <Resizer
                                        data-resizer-id={`${index}`}
                                        onMouseDown={this.onResizeStart(index)}
                                    />
                                ) : null}
                            </>
                        );
                    },
                )}
            </Wrapper>
        );
    }

    private setSizes = () => {
        let currSize = 0;
        const style = this.panels.reduce((currStyle, panel, index) => {
            currSize += panel.height;
            return `
                ${currStyle}

                [data-panel-id='${index}'] {
                    top: ${currSize - panel.height}px;
                    height: ${panel.height}px;
                }

                [data-resizer-id='${index}'] {
                    top: calc(${currSize}px - 10px);
                }
            `;
        }, '');

        this.styleElement.innerHTML = style;
    };

    private onTitleClick = (index: number) => () => {
        const { inactivePanelHeight, minPanelHeight } = this.props;
        if (this.panels[index].active) {
            makePanelInactive(index, this.panels, inactivePanelHeight, minPanelHeight);
        } else {
            makePanelActive(
                index,
                this.panels,
                this.parentHeight,
                inactivePanelHeight,
                minPanelHeight,
            );
        }
        this.setSizes();
        this.forceUpdate();
    };

    private onResizeStart = (index: number) => () => {
        this.resizeIndex = index;
        this.mouseDelta = 0;
        this.parentHeight = this.wrapperRef.current!.clientHeight;

        requestAnimationFrame(this.onResize);
        window.addEventListener('mousemove', this.onMouseMove);
        window.addEventListener('mouseup', this.onResizeEnd);
    };

    private onMouseMove = (e: MouseEvent) => {
        if (isChrome) {
            // It appears that Chrome returns mouse movement in device pixels rather than
            // CSS pixels, which is annoying to say the least. Here, we check if we are
            // running in Chrome, and apply the transformation from display pixel to CSS
            // pixel.
            this.mouseDelta += e.movementY / this.pixelRatio;
        } else {
            this.mouseDelta += e.movementY;
        }
    };

    private onResize = () => {
        if (this.resizeIndex !== -1) {
            const { resizeIndex } = this;

            this.mouseDelta = resize(
                resizeIndex,
                this.panels,
                this.mouseDelta,
                this.props.minPanelHeight,
            );
            this.setSizes();

            requestAnimationFrame(this.onResize);
        }
    };

    private onResizeEnd = () => {
        this.resizeIndex = -1;
        this.mouseDelta = 0;

        window.removeEventListener('mousemove', this.onMouseMove);
        window.removeEventListener('mouseup', this.onResizeEnd);
    };

    private onResizeParent = () => {
        const newParentHeight = this.wrapperRef.current!.clientHeight;
        resizeParent(this.panels, newParentHeight - this.parentHeight, 100);
        this.parentHeight = newParentHeight;
        this.setSizes();
    };
}
