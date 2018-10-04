import * as React from 'react';
import { Subscribe } from 'unstated';

import { TopicListener, TopicListenerType } from '~/components/containers/TopicListener';

import { Canvas } from '~/components/visualizer/Canvas';
import { Circle } from '~/components/visualizer/Circle';
import { Rectangle } from '~/components/visualizer/Rectangle';

import { ITurtlePose } from '~/types/standardTopics';

/**
 * Visualizer module. Display a ball using information from the /ball/pose ROS topic.
 */
export const Visualizer = () => {
    return (
        <Canvas>
            <Subscribe to={[TopicListener('/turtle1/pose')]}>
                {mapTurtlePose}
            </Subscribe>
        </Canvas>
    );
};

/**
 * 
 * @param rosout A TopicListener to access ROS log messages
 */
const mapTurtlePose = (pose: TopicListenerType<ITurtlePose>) => {
    return (
        <>
            <Rectangle
                x={0}
                y={0}
                width={560}
                height={560}
                fill={0xEEEEEE}
                />
            <Circle
                x={pose.state.currentValue ? pose.state.currentValue.x * 50 : 0}
                y={pose.state.currentValue ? 560 - pose.state.currentValue.y * 50 : 0}
                radius={6}
                fill={0xFF0000}
            />
        </>
    );
};