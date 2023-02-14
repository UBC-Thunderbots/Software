#include "software/physics/euclidean_to_wheel.h"

/**
     * Ramp the velocity over the given timestep and set the target velocity on the motor.
     *
     * NOTE: This function has no state.
     * Also NOTE: This function handles all electrical rpm to meters/second conversion.
     *
     * @param velocity_target The target velocity in m/s
     * @param velocity_current The current velocity m/s
     * @param time_to_ramp The time allocated for acceleration in seconds
     *
     */
WheelSpace_t rampWheelVelocity(const WheelSpace_t& current_wheel_velocity,
                               const EuclideanSpace_t& target_euclidean_velocity,
                               double max_allowable_wheel_velocity,
                               double allowed_acceleration,
                               const double& time_to_ramp);

WheelSpace_t rampWheelVelocity(const TbotsProto::DirectControlPrimitive& current_primitive,
                               const EuclideanSpace_t& target_euclidean_velocity,
                               double max_allowable_wheel_velocity,
                               double allowed_acceleration,
                               const double& time_to_ramp);