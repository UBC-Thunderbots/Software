import * as React from 'react';

import { Subscribe } from 'unstated';

import { TopicPublisher } from '~/components/containers/TopicPublisher';
import { ITurtleCmdVel } from '~/types/standardTopics';

// ROS Messages
const UP = {
    angular: {
        x: 0,
        y: 0,
        z: 0,
    },
    linear: {
        x: 2,
        y: 0,
        z: 0,
    },
};

const DOWN = {
    angular: {
        x: 0,
        y: 0,
        z: 0,
    },
    linear: {
        x: -2,
        y: 0,
        z: 0,
    },
};

const LEFT = {
    angular: {
        x: 0,
        y: 0,
        z: 2,
    },
    linear: {
        x: 0,
        y: 0,
        z: 0,
    },
};

const RIGHT = {
    angular: {
        x: 0,
        y: 0,
        z: -2,
    },
    linear: {
        x: 0,
        y: 0,
        z: 0,
    },
};

/**
 * Control module. Allows to send keyboard messages.
 */
export const Control = () => (
    <Subscribe to={[TopicPublisher]}>
        {mapControls}
    </Subscribe>
);

const mapControls = (controls: TopicPublisher<ITurtleCmdVel>) => {
    return (
        <div>
            <button onClick={() => controls.publish('/turtle1/cmd_vel', 'geometry_msgs/Twist', LEFT)}>Left</button>
            <button onClick={() => controls.publish('/turtle1/cmd_vel', 'geometry_msgs/Twist', UP)}>Up</button>
            <button onClick={() => controls.publish('/turtle1/cmd_vel', 'geometry_msgs/Twist', DOWN)}>Down</button>
            <button onClick={() => controls.publish('/turtle1/cmd_vel', 'geometry_msgs/Twist', RIGHT)}>Right</button>
        </div>
    );
};
