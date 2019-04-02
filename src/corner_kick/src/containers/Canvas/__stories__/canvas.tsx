/*
 * This file creates a story for Canvas
 */

import { storiesOf } from '@storybook/react';
import * as React from 'react';

import { StorybookStage } from 'SRC/components/StorybookStage';
import { BYTES_PER_SHAPE } from 'SRC/constants';

import { expectedSpritesheetFormat } from '../__fixtures__/spritesheet';
import { Canvas } from '../canvas';
import { CanvasManager } from '../canvasManager';

const stories = storiesOf('Canvas', module);

const data = new ArrayBuffer(500 * BYTES_PER_SHAPE + 1);
const dataView = new DataView(data);
dataView.setUint8(0, 0);

const canvasManager = new CanvasManager(expectedSpritesheetFormat);

stories.add('with animated sprites', () => (
    <StorybookStage width="80%" height="80%">
        <Canvas canvasManager={canvasManager} />
    </StorybookStage>
));

let j = 0; // We use this counter to add animation
const renderShapes = () => {
    // Modify each shape...
    for (let i = 0; i < 500; ++i) {
        dataView.setUint8(15 * i + 1, i % 6);
        dataView.setInt16(15 * i + 2, (i % 20) * 240);
        dataView.setInt16(15 * i + 4, Math.floor(i / 20) * 120);
        dataView.setInt16(15 * i + 6, 120);
        dataView.setInt16(15 * i + 8, 120);
        dataView.setInt16(15 * i + 10, (j / 100) % 3600);
        dataView.setUint8(15 * i + 12, 255);
        dataView.setUint8(15 * i + 13, (i * 100 + j) / 157);
        dataView.setUint8(15 * i + 14, (i * 100 + j) / 567);
        dataView.setUint8(15 * i + 15, (i * 100 + j) / 789);

        j++;
    }

    // And notify the CanvasManager of the change
    canvasManager.handleLayerMessage(data, () => {
        console.log('New layer detected');
    });

    // Wait 16ms and start again
    setTimeout(renderShapes, 16);
};

renderShapes();
