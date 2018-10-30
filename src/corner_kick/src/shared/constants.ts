/**
 * @fileoverview This file specifies constants and magic values
 * used throughout this application. Some of these values might
 * be later moved to a configuration file once implemented.
 */

// ELECTRON CONSTANTS //

/**
 * @description Main window default width
 */
export const DEFAULT_WINDOW_WIDTH = 900;

/**
 * @description Main window default height
 */
export const DEFAULT_WINDOW_HEIGHT = 680;

/**
 * @description Local server URL for development
 */
export const DEV_SERVER_URL = 'http://localhost:8080/';

/**
 * @description Renderer start file location for production
 */
export const RENDERER_START_FILE = './build/index.html';

// ROS CONSTANTS //

/**
 * @description The URL to use when connecting to ROS bridge.
 */
export const ROS_URL = 'ws:localhost:9090';

// STYLING CONSTANTS //

/**
 * @description The visualizer background color
 */
export const VISUALIZER_BACKGROUND_COLOR = 0x417f11;

/**
 * @description Color of the field lines
 */
export const FIELD_LINE_COLOR = 0xffffff;

/**
 * @description Field line stroke width
 */
export const FIELD_LINE_WIDTH = 1;

/**
 * @description Starting pane height
 */
export const DEFAULT_PANE_HEIGHT = 300;

// VISUALIZER CONSTANT //

/**
 * @description Visual multiply factor for all elements of the visualizer.
 */
export const MULTIPLY_FACTOR = 50;

/**
 * @description The world size of the visualizer
 */
export const VISUALIZER_WORLD_SIZE = 1000;

// TIME CONSTANTS //

/**
 * @description The time we wait until we check if there are any
 * new topics to subscribe to.
 */
export const TIME_FOR_TOPIC_REFRESH = 10000;

/**
 * @description The shortest amount of time in millis we allow
 * between UI updates. This is to avoid consuming too much resources
 * by generating unnecessary UI updates.
 */
export const MIN_TIME_BETWEEN_UI_UPDATES = 32;

/**
 * @description Max number of ROS messages to store per topic.
 */
export const MAX_STORED_ROS_MESSAGES = 100;
