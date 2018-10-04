import { Container } from "unstated";

import { ROSConnector } from "~/service/RosConnector";

/**
 * A TopicPublisher allows us to send message at a given topic
 * @param topic the name of the topic to listen to
 */
export class TopicPublisher<T> extends Container<{}> {

    private ros: ROSConnector;
    
    public constructor() {
        super();

        this.ros = ROSConnector.Instance;
    }

    /**
     * Publishes a message to the given topic
     */
    public publish = (topic: string, messageType: string, message: T) => {
        this.ros.publish(topic, messageType, message);
    }
}