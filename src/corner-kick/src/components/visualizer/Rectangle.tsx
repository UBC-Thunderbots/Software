import * as PIXI from 'pixi.js';
import { ContainerProperties, CustomPIXIComponent } from 'react-pixi-fiber';

interface IRectangleProps extends ContainerProperties {
    fill?: number;
    stroke?: number;
    strokeWidth?: number;
}

const TYPE = 'Rectangle';
const behavior = {
    customApplyProps: (instance: PIXI.Graphics, oldProps: IRectangleProps, newProps: IRectangleProps) => {
        const { fill, stroke, strokeWidth, width, height } = newProps;

        instance.x = newProps.x || 0;
        instance.y = newProps.y || 0;

        instance.clear();

        if (fill !== undefined) {
            instance.beginFill(fill);
        }

        if (stroke !== undefined && strokeWidth !== undefined) {
            instance.lineStyle(strokeWidth, stroke);
        }

        instance.drawRect(0, 0, width || 0, height || 0);
        instance.endFill();
    },
    customDisplayObject: (props: {}) => new PIXI.Graphics(),
};

/**
 * Renders a rectangle in a canvas context
 */
export const Rectangle =  CustomPIXIComponent(behavior, TYPE);