import * as storage from 'electron-json-storage';
import { Container } from 'unstated';

const persistContainers: {
  [key: string]: ReturnType<typeof generatePersist>;
} = {};

/**
 * A Persist container allows us to retrieve and set settings to persist between
 * application runs.
 * @param key the name of settings file to access
 */
export const Persist = (key: string) => {
  if (persistContainers[key] === undefined) {
    persistContainers[key] = generatePersist(key);
  }

  return persistContainers[key];
};

/**
 * Generates a persist container for a settings file.
 * @param key the name of settings file to access
 */
const generatePersist = (key: string) =>
  class PersistContainer extends Container<any> {
    public state = {};

    public constructor() {
      super();

      // Retrieve existing settings when we first initialize.
      storage.get(key, (error, data) => {
        if (error) {
          throw error;
        } else {
          this.setState(data);
        }
      });
    }

    /**
     * Allows us to persist new values between application runs.
     * @param data the data to persist
     */
    public set = (data: any) => {
      storage.set(key, { ...this.state, ...data }, (error) => {
        if (error) {
          throw error;
        }
      });

      // We modify the state so that other users of this container
      // will receive the new value.
      this.setState(data);
    };
  };
