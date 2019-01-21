import * as _ from 'lodash';
import * as React from 'react';

import styled from 'SRC/utils/styled-components';

import { IInternalPanelProps, Panel } from './Panel';

const Wrapper = styled.div`
    position: relative;
    width: 100%;
    height: 100%;
`;

const Resizer = styled.div`
    position: absolute;
    width: 100%;
    height: 10px;

    cursor: row-resize;
`;

interface IResizeablePanelsProps {
    children: React.ReactElement<typeof Panel> | Array<React.ReactElement<typeof Panel>>;
}

export class ResizeablePanels extends React.Component<IResizeablePanelsProps> {
    private wrapperRef: React.RefObject<HTMLDivElement>;
    private styleElement: HTMLStyleElement;

    private resizeIndex = -1;
    private mouseDelta = 0;
    private childrenSize: number[];

    constructor(props: IResizeablePanelsProps) {
        super(props);

        this.wrapperRef = React.createRef();
    }

    public componentDidMount() {
        this.styleElement = document.createElement('style');
        this.styleElement.type = 'text/css';

        const head = document.querySelector('head');
        head!.appendChild(this.styleElement);

        const count = React.Children.count(this.props.children);
        this.childrenSize = React.Children.map(this.props.children, (child) =>
            Math.round(100 / count),
        );

        this.setSizes();
    }

    public render() {
        const { children } = this.props;

        const count = React.Children.count(children);

        return (
            <Wrapper ref={this.wrapperRef}>
                {React.Children.map(
                    children,
                    (child: React.ReactElement<IInternalPanelProps>, index) => {
                        return (
                            <>
                                {React.cloneElement(child, {
                                    active: true,
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

    private onTitleClick = (index: number) => () => {
        //
    };

    private setSizes = () => {
        let currSize = 0;
        const style = this.childrenSize.reduce((currStyle, size, index) => {
            currSize += size;
            return `
                ${currStyle}

                [data-panel-id='${index}'] {
                    height: ${size}%;
                }

                [data-resizer-id='${index}'] {
                    top: calc(${currSize}% - 10px);
                }
            `;
        }, '');

        this.styleElement.innerHTML = style;
    };

    private onResizeStart = (index: number) => () => {
        this.resizeIndex = index;
        this.mouseDelta = 0;

        setTimeout(this.onResize, 100);
        window.addEventListener('mousemove', this.onMouseMove);
        window.addEventListener('mouseup', this.onResizeEnd);
    };

    private onMouseMove = (e: MouseEvent) => {
        this.mouseDelta += e.movementY;
    };

    private onResize = () => {
        if (this.resizeIndex !== -1) {
            const { childrenSize, resizeIndex } = this;

            childrenSize[resizeIndex] += Math.round((this.mouseDelta / 920) * 100);
            childrenSize[resizeIndex + 1] -= Math.round((this.mouseDelta / 920) * 100);

            this.setSizes();

            this.mouseDelta = 0;

            setTimeout(this.onResize, 100);
        }
    };

    private onResizeEnd = () => {
        this.resizeIndex = -1;
        this.mouseDelta = 0;

        window.removeEventListener('mousemove', this.onMouseMove);
        window.removeEventListener('mouseup', this.onResizeEnd);
    };
}
