/**
 * @fileoverview Defines an interface to access messages received
 * for a particular ROS topic.
 */

import * as _ from 'lodash';
import { Container } from 'unstated';

import { MAX_STORED_ROS_MESSAGES, MIN_TIME_BETWEEN_UI_UPDATES } from 'SHARED/constants';
import { ROSConnector } from 'SHARED/ros/RosConnector';

/**
 * @description Interface that defines the format of our topic container.
 */
interface ITopicListenerState<T = any> {
    /**
     * @description The topic we are listening from
     */
    topic: string;

    /**
     * @description The last value we received from this topic
     */
    currentValue: T;

    /**
     * @description The last 100 values received for this topic
     */
    values: T[];
}

const topicContainers: {
    [topic: string]: ReturnType<typeof generateTopicListener>;
} = {};

/**
 * @description Type information for a TopicListener
 */
export type TopicListenerType<T> = Container<ITopicListenerState<T>>;

/**
 * @description A TopicListener allows us to be notified when a
 * new message is received on a given ROS topic. The part of the
 * UI using this listener will be invalidated and automatically updated.
 * @param topic the name of the topic to listen to
 */
export const TopicListener = (topic: string) => {
    if (topicContainers[topic] === undefined) {
        topicContainers[topic] = generateTopicListener(topic);
    }

    return topicContainers[topic];
};

/**
 * @description Generates a topic listener for a given ROS topic.
 * @param topic the name of the topic to listen to
 */
const generateTopicListener = (topic: string) =>
    class ROSTopicListener extends Container<ITopicListenerState> {
        private ros: ROSConnector;

        public constructor() {
            super();

            this.state = {
                topic,
                currentValue: undefined,
                values: [],
            };

            this.ros = ROSConnector.Instance;

            // Subscribe to a topic. Notice we use a throttle function. We want to
            // limit the number of times the callback function
            // is called to reduce the number of UI updates, as it impacts performance.
            this.ros.subscribeToTopic(
                topic,
                _.throttle(this.onNewMessage, MIN_TIME_BETWEEN_UI_UPDATES),
            );
        }

        /**
         * @description Updates the state when a new message is received.
         * This will cause the UI to update as well.
         */
        private onNewMessage = (message: any) => {
            this.setState({
                currentValue: message,
                values: [
                    message,
                    // We limit the number of past values
                    ...this.state.values.slice(0, MAX_STORED_ROS_MESSAGES),
                ],
            });
        };
    };
