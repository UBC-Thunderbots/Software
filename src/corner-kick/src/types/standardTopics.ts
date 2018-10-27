
/**
 * ROS topic used for logging purposes
 */
export interface IROSOut {
    file: string;
    function: string;
    level: number;
    line: number;
    msg: string;
    name: string;
    topics: string[];
}