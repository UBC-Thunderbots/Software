import { Container } from "unstated";
import { ROSService } from "../../service/ros";

interface ITopicContainerState {
    topic: string;
    currentValue: any;
    values: any[];
}

const generateTopicContainer = (topic: string) => (
    class ROSTopicContainer extends Container<ITopicContainerState> {

        private ros: ROSService;
        
        public constructor() {
            super();

            this.state = {
                currentValue: undefined,
                topic,
                values: [],
            };

            this.ros = ROSService.Instance;
            this.ros.subscribeToTopic(topic, this.onNewMessage);
        }

        private onNewMessage = (message: any) => {
            this.setState({
                currentValue: message,
                values: [
                    message,
                    ...this.state.values,
                ],
            });
        }
    }
);

export type TopicContainerType = Container<ITopicContainerState>;

const topicContainers: {[topic: string]: ReturnType<typeof generateTopicContainer>} = {};
export const TopicContainer = (topic: string) => {
    if(topicContainers[topic] === undefined) {
        topicContainers[topic] = generateTopicContainer(topic);
    }

    return topicContainers[topic];
}