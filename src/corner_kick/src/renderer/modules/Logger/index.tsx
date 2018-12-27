/**
 * @fileoverview Defines a Corner Kick module used to display
 * log messages sent to the ROS topic /rosout.
 */

import * as React from 'react';

import { Subscribe } from 'unstated';

import {
    TopicListener,
    TopicListenerType,
} from 'RENDERER/components/containers/TopicListener';
import { IROSOut } from 'SHARED/ros/types/standardTopics';
import { LogMessage } from './LogMessage';

const ROSOUT_TOPIC = '/rosout';

/**
 * @description Logging module. Displays the /rosout ROS topic,
 * which is used by all nodes to display debugging information.
 */
export const Logger = () => {
    // Here, we subscribe to the /rosout topic. The function mapLogMessages will
    // automatically be called when
    // a new message is sent to /rosout, allowing us to update our UI.
    return <Subscribe to={[TopicListener(ROSOUT_TOPIC)]}>{mapLogMessages}</Subscribe>;
};

/**
 * @description Function that maps messages received from /rosout
 * to React components.
 * @param rosout A TopicListener to access ROS messages from /rosout
 */
const mapLogMessages = (rosout: TopicListenerType<IROSOut>) => {
    // Build a list UI elements for each /rosout log message.
    const logs = rosout.state.values.map((logMessage) => (
        <LogMessage key={new Date().getTime()} message={logMessage.msg} />
    ));

    // We return this list for it to be displayed in our UI
    return <div>{logs}</div>;
};
