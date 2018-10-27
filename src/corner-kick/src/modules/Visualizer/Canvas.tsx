import * as _ from 'lodash';
import * as PIXI from 'pixi.js';
import * as Viewport from 'pixi-viewport';
import * as React from 'react';
import * as storage from 'electron-json-storage';

import { IResizeEntry, ResizeSensor } from "@blueprintjs/core";
import { Field } from '~/modules/Visualizer/components/Field';

/**
 * A React element to display a canvas.
 */
export class Canvas extends React.Component {

    public state = {
        width: 0,
        height: 0,
    }

    private canvasRef: React.RefObject<HTMLCanvasElement>;
    private app: PIXI.Application;
    private viewport : Viewport

    constructor(props: {}) {
        super(props);
        this.canvasRef = React.createRef();
    }

    public componentDidMount() {
        const canvas = this.canvasRef.current!;

        this.app = new PIXI.Application({
            autoStart: true,
            antialias: true,
            backgroundColor: 0x417F11,
            height: 0,
            width: 0,
            view: canvas
        });

        this.viewport = new Viewport({
            screenWidth: window.innerWidth,
            screenHeight: window.innerHeight,
            worldWidth: 1000,
            worldHeight: 1000,
         
            interaction: (this.app.renderer as any).interaction
        });

        this.viewport
            .drag()
            .pinch()
            .wheel()
            .decelerate();

        storage.get('visualizer', (error, data: any) => {
            this.viewport.moveCenter(new PIXI.Point(data.x || 0, data.y || 0));
        })

        this.viewport.on("moved-end", () => {
            storage.set('visualizer', {
                x: this.viewport.center.x,
                y: this.viewport.center.y,
            },(error) => {
                if(error) {
                    throw error;
                }
            });
        });

        this.app.stage.addChild(this.viewport);

        this.viewport.addChild(Field());
    }

    public render() {
        return (
            <ResizeSensor observeParents={true} onResize={_.debounce(this.onResize, 16)}>
                <canvas ref={this.canvasRef} width={this.state.width} height={this.state.height}/>
            </ResizeSensor>
        );
    }

    public onResize = (entries: IResizeEntry[]) => {
        if(entries[1] !== undefined) {
            const {width, height} = entries[1].contentRect;
            this.setState({width, height});
            this.app.renderer.resize(width, height);
            this.viewport.resize(width, height);
        }
    }
}