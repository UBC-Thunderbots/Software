/**
 * @fileoverview Defines an interface to publish messages
 * to a given ROS topic.
 */

import { Container } from 'unstated';

import { ROSConnector } from 'SHARED/ros/RosConnector';

/**
 * @description A TopicPublisher allows us to send message at a given topic
 * @param topic the name of the topic to listen to
 */
export class TopicPublisher<T> extends Container<{}> {
    private ros: ROSConnector;

    public constructor() {
        super();

        this.ros = ROSConnector.Instance;
    }

    /**
     * @description Publishes a message to the given topic
     * @param topic The topic to publish to
     * @param messsageType The message type of the topic
     * @param message The message to send
     */
    public publish = (topic: string, messageType: string, message: T) => {
        this.ros.publish(topic, messageType, message);
    };
}
