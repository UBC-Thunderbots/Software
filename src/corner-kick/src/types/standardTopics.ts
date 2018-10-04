
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

export interface ITurtlePose {
    angular_velocity: number;
    linear_velocity: number;
    theta: number;
    x: number;
    y: number;
}

export interface ITurtleCmdVel {
    linear: {
        x: number;
        y: number;
        z: number;
    };
    angular: {
        x: number;
        y: number;
        z: number;
    };
}