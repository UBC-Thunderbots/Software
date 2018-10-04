import * as PIXI from 'pixi.js';
import { ContainerProperties, CustomPIXIComponent } from 'react-pixi-fiber';

interface IRectangleProps extends ContainerProperties {
    fill?: number;
    stroke?: number;
    strokeWidth?: number;
    radius: number;
    width: never;
    height: never;
}

const TYPE = 'Circle';
const behavior = {
    customApplyProps: (instance: PIXI.Graphics, oldProps: IRectangleProps, newProps: IRectangleProps) => {
        const { fill, stroke, strokeWidth, radius } = newProps;

        instance.x = newProps.x || 0;
        instance.y = newProps.y || 0;

        instance.clear();

        if (fill !== undefined) {
          instance.beginFill(fill);
        }

        if (stroke !== undefined && strokeWidth !== undefined) {
          instance.lineStyle(strokeWidth, stroke);
        }

        instance.drawCircle(0, 0, radius || 0);
        instance.endFill();
    },
    customDisplayObject: (props: {}) => new PIXI.Graphics(),
};

/**
 * Renders a circle in a canvas context
 */
export const Circle =  CustomPIXIComponent(behavior, TYPE);