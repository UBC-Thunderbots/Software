import * as PixiViewport from 'pixi-viewport';
import { ContainerProperties, CustomPIXIComponent } from 'react-pixi-fiber';

interface IViewportProps extends ContainerProperties {
    screenWidth: number;
    screenHeight: number;
    worldWidth: number;
    worldHeight: number;
}

const TYPE = 'Viewport';
const behavior = {
    customApplyProps: (instance: PixiViewport , oldProps: IViewportProps, newProps: IViewportProps) => {
      instance.x = newProps.x || 0;
      instance.y = newProps.y || 0;
      instance.width = newProps.width || 0;
      instance.height = newProps.height || 0;
    },
    customDisplayObject: (props: IViewportProps): PixiViewport => {
        const viewport = new PixiViewport({
            screenHeight: props.screenWidth,
            screenWidth: props.screenWidth,
            worldHeight: props.worldHeight,
            worldWidth: props.worldWidth,
        });

        viewport
            .drag()
            .wheel();

        return viewport;
    },
};

/**
 * A viewport for a Pixi.js context. Allows for drag and zoom.
 */
export const Viewport =  CustomPIXIComponent(behavior as any, TYPE) as React.ReactType<IViewportProps>;