/*
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

const Wrapper = styled.div`
    position: relative;
    width: 100%;
    height: 100%;

    overflow: hidden;
`;

const Resizer = styled.div`
    position: absolute;
    width: 100%;
    height: 10px;

    cursor: row-resize;
`;

interface IResizeablePanelsProps {
    minPanelSize: number;
    children: React.ReactElement<typeof Panel> | Array<React.ReactElement<typeof Panel>>;
}

export class ResizeablePanels extends React.Component<IResizeablePanelsProps> {
    private wrapperRef: React.RefObject<HTMLDivElement>;
    private styleElement: HTMLStyleElement;

    private resizeIndex = -1;
    private mouseDelta = 0;
    private parentHeight = 0;
    private panels: IPanel[];
    private pixelRatio: number;

    private resizeObserver: ResizeObserver;

    constructor(props: IResizeablePanelsProps) {
        super(props);

        this.wrapperRef = React.createRef();

        this.panels = getInitialSize(React.Children.count(this.props.children), 100);
    }

    public componentDidMount() {
        this.parentHeight = this.wrapperRef.current!.clientHeight;
        this.resizeObserver = new ResizeObserver(this.onResizeParent);
        this.resizeObserver.observe(this.wrapperRef.current!);

        this.pixelRatio = window.devicePixelRatio;

        this.styleElement = document.createElement('style');
        this.styleElement.type = 'text/css';

        const head = document.querySelector('head');
        head!.appendChild(this.styleElement);

        this.panels = getInitialSize(
            React.Children.count(this.props.children),
            this.parentHeight,
        );
        this.setSizes();
    }

    public componentWillUnmount() {
        this.resizeObserver.unobserve(this.wrapperRef.current!);
    }

    public render() {
        const { children, minPanelSize } = this.props;

        const count = React.Children.count(children);

        return (
            <Wrapper ref={this.wrapperRef} style={{ minHeight: count * minPanelSize }}>
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
            currSize += panel.size;
            return `
                ${currStyle}

                [data-panel-id='${index}'] {
                    top: ${currSize - panel.size}px;
                    height: ${panel.size}px;
                }

                [data-resizer-id='${index}'] {
                    top: calc(${currSize}px - 10px);
                }
            `;
        }, '');

        this.styleElement.innerHTML = style;
    };

    private onTitleClick = (index: number) => () => {
        if (this.panels[index].active) {
            makePanelInactive(index, this.panels, 32);
        } else {
            makePanelActive(
                index,
                this.panels,
                this.parentHeight,
                this.props.minPanelSize,
            );
        }
        this.setSizes();
        this.forceUpdate();
    };

    private onResizeStart = (index: number) => () => {
        this.resizeIndex = index;
        this.mouseDelta = 0;
        this.parentHeight = this.wrapperRef.current!.clientHeight;

        console.log(this.parentHeight);

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
                this.props.minPanelSize,
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
