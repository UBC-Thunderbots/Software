import * as storage from 'electron-json-storage';
import { Container } from "unstated";

const persistContainers: {[key: string]: ReturnType<typeof generatePersist>} = {};

/**
 * A TopicListener allows us to be notified when a
 * new message is received on a given ROS topic. The part of the UI using this listener will be invalidated and automatically updated.
 * @param topic the name of the topic to listen to
 */
export const Persist = (key: string) => {
    if(persistContainers[key] === undefined) {
        persistContainers[key] = generatePersist(key);
    }

    return persistContainers[key];
};

/**
 * Generates a topic listener for a given ROS topic.
 * @param topic the name of the topic to listen to
 */
const generatePersist = (key: string) => (
    class PersistContainer extends Container<{}> {
        
        public constructor() {
            super();

            this.state = {};

            storage.get(key, (error, data) => {
                if(error) {
                    throw error;
                }

                this.setState(data);
              });
        }

        public set = (data: any) => {
            storage.set(key, {...this.state, ...data}, (error) => {
                if(error) {
                    throw error;
                }
            });
            this.setState(data);
        }
    }
);