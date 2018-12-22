/**
 * @fileoverview Allows to connect to ROS via ROSBridge and
 * provides access to all ROS topics.
 */

import ROSLib from 'roslib';

import { ROS_URL, TIME_FOR_TOPIC_REFRESH } from 'SHARED/constants';

/**
 * @description Handles the communication with ROSBridge. Namely handles connecting
 * to ROS, subscribing to topics and receiving messages from topics.
 * Singleton.
 *
 * @see TopicListener.tsx for usage.
 */
export class ROSConnector {
    private static instance: ROSConnector;

    private ros: any; // TODO: create typing information for the ROS object

    /**
     * @description Stores the topics we are currently registered with. This helps us
     * avoids subscribing to a topic we are already subscribed with.
     */
    private topics: string[] = [];

    /**
     * @description Stores the callback functions for each topic. When a new message
     * arrives for a given topic, we call the functions stored in this dictionary.
     */
    private topicsSubscriptions: { [topic: string]: [(topic: any) => void] } = {};

    /**
     * @description Returns an instance of ROSConnector
     */
    public static get Instance() {
        return this.instance || (this.instance = new this());
    }

    private constructor() {
        this.ros = new ROSLib.Ros({
            url: ROS_URL,
        });
        this.subscribeToAllTopics();
    }

    /**
     * @description Subscribe to a given topic. When new messages are
     * received on the topic, the callback will be called.
     * @param topic the topic to subscribe to
     * @param callback the callback function to call when a new topic is received
     */
    public subscribeToTopic = (topic: string, callback: (topic: any) => void) => {
        // We initialize a new array if this is the first callback for a given topic
        if (this.topicsSubscriptions[topic] === undefined) {
            this.topicsSubscriptions[topic] = [callback];
        } else {
            this.topicsSubscriptions[topic].push(callback);
        }
    };

    /**
     * @description Unsubscribe from a given topic.
     * @param topic the topic to unsubscribe from
     * @param callback the callback function to remove
     */
    public unsubscribeToTopic = (topic: string, callback: (topic: any) => void) => {
        if (this.topicsSubscriptions[topic] !== undefined) {
            const index = this.topicsSubscriptions[topic].indexOf(callback);
            this.topicsSubscriptions[topic].splice(index, 1);
        }
    };

    /**
     * @description Publishes a message to a given ROS topic.
     * @param name the name of the topic
     * @param messageType the message type of the topic
     * @param message the message to send
     */
    public publish = (name: string, messageType: string, message: any) => {
        const topic = new ROSLib.Topic({
            messageType,
            name,
            ros: this.ros,
        });

        topic.publish(message);
    };

    /**
     * @description Internal method to subscribe to all ROS topics. This
     * is the method that actually "subscribes" in a ROS sense. We do this
     * to create an abstraction layer between ROS and the rest of our application
     */
    private subscribeToAllTopics = () => {
        // Request ROS to give us a list of all topics.
        this.ros.getTopics(this.onTopicsReceived);

        // Every 10 seconds, ask for a new list from ROS. If new topics
        // are added during runtime,
        // we will pick them up.
        setTimeout(this.subscribeToAllTopics, TIME_FOR_TOPIC_REFRESH);
    };

    /**
     * @description This internal method subscribes (in a ROS sense) to every topic found.
     * When a new message is received, it calls onMessageReceived, which does the actual
     * act of notifying the rest of the application.
     */
    private onTopicsReceived = (response: { topics: string[]; types: string[] }) => {
        const newTopics: string[] = [];

        response.topics.forEach((element, index) => {
            // We only subscribe to topics we have not seen yet
            if (this.topics.indexOf(element) === -1) {
                const topic = new ROSLib.Topic({
                    messageType: response.types[index],
                    name: element,
                    ros: this.ros,
                });
                topic.subscribe(this.onMessageReceived(element));
            }
            // We add the topic to the list of topics seen
            newTopics.push(element);
        });

        this.topics = newTopics;
    };

    /**
     * @description Method called when a new message is received from a ROS topic.
     * Does the actual work of calling the user-provided callback functions.
     */
    private onMessageReceived = (name: string) => (message: any) => {
        if (this.topicsSubscriptions[name] !== undefined) {
            this.topicsSubscriptions[name].forEach((callback) => callback(message));
        } else {
            // We should have a subscription, if not this method would not be called.
            console.error('Received message but we did not have a subscription for it');
        }
    };
}
