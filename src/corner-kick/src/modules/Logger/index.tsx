import * as React from 'react';

import { Subscribe } from 'unstated';

import { TopicListener, TopicListenerType } from '~/components/containers/TopicListener';
import { LogMessage } from './LogMessage';
import { IROSOut } from '~/types/standardTopics';

/**
 * Logging module. Displays the /rosout ROS topic, which is used by all nodes to display debugging information.
 */
export const Logger = () => {

    // Here, we subscribe to the /rosout topic. The function mapLogMessages will automatically be called when 
    // a new message is sent to /rosout, allowing us to update our UI.
    return (
        <Subscribe to={[TopicListener('/rosout')]}>
            {mapLogMessages}
        </Subscribe>
    );
};

/**
 * 
 * @param rosout A TopicListener to access ROS log messages
 */
const mapLogMessages = (rosout: TopicListenerType<IROSOut>) => {

    // Build a list UI elements for each /rosout log message.
    const logs = rosout.state.values.map((logMessage) => (
        <LogMessage message={logMessage.msg} />
    ));

    // We return this list for it to be displayed in our UI
    return (
        <div>
            {logs}
        </div>
    );
};
