/**
 * This struct represents robot constants
 */
typedef struct RobotConstants
{
    // The mass of the entire robot [kg]
    float mass;
    // The moment of inertia of the entire robot [kg m^1]
    float moment_of_inertia;
    // The maximum radius of the robot [m]
    float robot_radius;
    // The maximum jerk this robot may safely undergo [m/s^2]
    float jerk_limit;
} RobotConstants_t;

/**
 * This struct holds the state of the controller.
 * This is a carryover from legacy code, and should be deleted when the controller is
 * replaced.
 */
typedef struct ControllerState
{
    float last_applied_acceleration_x;
    float last_applied_acceleration_y;
    float last_applied_acceleration_angular;
} ControllerState_t
