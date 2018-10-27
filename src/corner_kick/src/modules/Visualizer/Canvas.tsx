import * as PIXI from 'pixi.js';

import * as storage from 'electron-json-storage';
import * as _ from 'lodash';
import * as Viewport from 'pixi-viewport';
import * as React from 'react';

import { IResizeEntry, ResizeSensor } from '@blueprintjs/core';
import { Field } from '~/modules/Visualizer/components/Field';

/**
 * The visualizer object. Consists of a WebGL enabled canvas, a 2D camera, and
 * the various objects to visualize (aka. field, ball, robots, etc.)
 */
export class Canvas extends React.Component {
  public state = {
    width: 0,
    height: 0,
  };

  private canvasRef: React.RefObject<HTMLCanvasElement>;
  private app: PIXI.Application;
  private viewport: Viewport;

  constructor(props: {}) {
    super(props);

    // A React ref allows us to access the HTML node once it is created.
    this.canvasRef = React.createRef();
  }

  public componentDidMount() {
    const canvas = this.canvasRef.current!;

    // Create a PIXI.js app. PIXI.js acts as an abstraction layer above the WebGL context.
    // TODO make background color parameterizable.
    this.app = new PIXI.Application({
      autoStart: true,
      antialias: true,
      backgroundColor: 0x417f11,
      height: 0,
      width: 0,
      view: canvas,
    });

    // Create the 2D camera
    // TODO make world size parameterizable
    this.viewport = new Viewport({
      screenWidth: window.innerWidth,
      screenHeight: window.innerHeight,
      worldWidth: 1000,
      worldHeight: 1000,

      interaction: (this.app.renderer as any).interaction,
    });

    this.viewport
      .drag()
      .pinch()
      .wheel();

    // We persist the past position of the camera between application runs.
    // Retrieve the position and set it.
    storage.get('visualizer', (error, data: any) => {
      this.viewport.moveCenter(new PIXI.Point(data.x || 0, data.y || 0));
    });

    // Persist the position when we move the camera.
    this.viewport.on('moved-end', () => {
      storage.set(
        'visualizer',
        {
          x: this.viewport.center.x,
          y: this.viewport.center.y,
        },
        (error) => {
          if (error) {
            throw error;
          }
        },
      );
    });

    this.app.stage.addChild(this.viewport);

    // TODO make this parameterizable
    this.viewport.addChild(Field());
  }

  public render() {
    // We use a resize sensor to resize the canvas when the parent pane or window is resized.
    return (
      <ResizeSensor
        observeParents={true}
        onResize={_.debounce(this.onResize, 16)}
      >
        <canvas
          ref={this.canvasRef}
          width={this.state.width}
          height={this.state.height}
        />
      </ResizeSensor>
    );
  }

  /**
   * Function that handles the canvas resize.
   */
  public onResize = (entries: IResizeEntry[]) => {
    if (entries[1] !== undefined) {
      const { width, height } = entries[1].contentRect;
      this.setState({ width, height });
      this.app.renderer.resize(width, height);
      this.viewport.resize(width, height);
    }
  };
}
