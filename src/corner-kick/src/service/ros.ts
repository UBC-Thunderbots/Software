import ROSLib from 'roslib';

const ROS_URL = 'ws:localhost:9090';

let ros: any;
window.addEventListener('unload', () => {
    ros.close();
})

export class ROSService {

    private static instance: ROSService;
    private topics: string[] = [];
    private topicsSubscription: {[topic: string]: [(topic: any) => void]} = {};
        
    public static get Instance() {
        return this.instance || (this.instance = new this());
    }

    private constructor() {
        ros = new ROSLib.Ros({
            url: ROS_URL,
        });
        this.subscribeToTopics();
    }

    public subscribeToTopic = (topic: string, callback: (topic: any) => void) => {
        if(this.topicsSubscription[topic] === undefined) {
            this.topicsSubscription[topic] = [callback];
        } else {
            this.topicsSubscription[topic].push(callback);
        }
    }

    public unsubscribeToTopic = (topic: string, callback: (topic: any) => void) => {
        if(this.topicsSubscription[topic] !== undefined) {
            const index = this.topicsSubscription[topic].indexOf(callback);
            this.topicsSubscription[topic].splice(index, 1);
        }
    }

    private subscribeToTopics = () => {
        ros.getTopics((response: {topics: string[], types: string[]}) => {
            const newTopics: string[] = [];
            response.topics.forEach((element, index) => {
                if(this.topics.indexOf(element) === -1) {
                    const topic = new ROSLib.Topic({
                        messageType: response.types[index],
                        name: element,
                        ros,
                    });
                    topic.subscribe(this.onMessageReceived(element));
                }
                newTopics.push(element);
            });
            this.topics = newTopics;
        });

        setTimeout(this.subscribeToTopics, 10000);
    }

    private onMessageReceived = (name: string) => (message: any) => {
        if(this.topicsSubscription[name] !== undefined) {
            this.topicsSubscription[name].forEach((callback) => callback(message));
        }
    }
}