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
} ControllerState_t;
