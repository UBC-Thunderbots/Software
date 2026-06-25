#pragma once
#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"
#include "software/embedded/motion_control/orientation_controller.h"
#include "software/embedded/motion_control/position_controller.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

class PrimitiveExecutor
{
   public:
    /**
     * Constructor
     * @param robot_constants The robot constants for the robot which uses this primitive
     * executor
     */
    explicit PrimitiveExecutor(const robot_constants::RobotConstants& robot_constants);

    /**
     * Update primitive executor with a new Primitive
     * @param primitive_msg The primitive to start
     */
    void updatePrimitive(const TbotsProto::Primitive& primitive_msg);

    /**
     * Update primitive executor with the state of the robot
     *
     * @param state The current robot state
     */
    void updateState(const RobotState& state);

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     *
     * @param status The status of the primitive executor, set to false if current
     * primitive is a Stop primitive
     * @param delta_time The elapsed time since the last primitive step
     *
     * @returns DirectControlPrimitive The direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        TbotsProto::PrimitiveExecutorStatus& status, const Duration& delta_time);

    // When true, the robot is constrained to only translate along its local x-axis
    // (forwards/backwards) and rotate -- it never strafes sideways (local y). While
    // moving, the robot faces its direction of travel along the planned path; once it is
    // within FORWARD_ONLY_FINAL_ROTATION_DISTANCE_M of the destination it rotates to the
    // primitive's requested final orientation. This models the robot as a non-holonomic
    // (unicycle) vehicle, which is useful when sideways motion is unreliable.
    //
    // The 2D position path itself is still planned holonomically by the AI; this flag
    // only changes how that path is executed on the robot. Flip to false to restore the
    // default holonomic (free-strafing) behavior.
    static constexpr bool ENABLE_FORWARD_ONLY_MOTION = true;

   private:
    /*
     * Compute the next target linear _local_ velocity the robot should have.
     * @param delta_time The elapsed time since last time step
     *
     * @returns Vector The target linear _local_ velocity
     */
    Vector stepTargetLinearVelocity(const Duration& delta_time);

    /*
     * Compute the next target angular velocity the robot should have.
     * @param delta_time The elapsed time since last time step
     *
     * @returns AngularVelocity The target angular velocity
     */
    AngularVelocity stepTargetAngularVelocity(const Duration& delta_time);

    /*
     * Compute the target angular velocity used in forward-only motion mode.
     *
     * While the robot is travelling, it rotates to face a pure-pursuit look-ahead point
     * on the planned path (so it steers along the path and back onto it, only ever
     * needing to drive forwards/backwards). Once it is near the destination, it rotates
     * to the primitive's requested final orientation instead. The returned angular
     * velocity comes from a saturated proportional controller toward that target
     * orientation, which settles without overshoot/oscillation. The value is unclamped;
     * stepTargetAngularVelocity applies the shared max-speed/max-acceleration limits.
     *
     * @returns AngularVelocity The (unclamped) target angular velocity
     */
    AngularVelocity stepForwardOnlyTargetAngularVelocity();

    /**
     * Sends the position, local velocity, and local acceleration to PlotJuggler.
     *
     * @param target_local_velocity The local velocity being sent to the next direct
     * control primitive
     * @param delta_time Used to calculate acceleration.
     */
    void sendLinearMotionToPlotJuggler(const Vector& target_local_velocity,
                                       const Duration& delta_time) const;

    /**
     * Records the velocities commanded this step so the next step can measure the
     * commanded (tick-to-tick) acceleration. Call on every code path that commands a
     * velocity without going through stepTargetLinearVelocity/stepTargetAngularVelocity.
     *
     * @param local_velocity The local velocity commanded this step
     * @param angular_velocity The angular velocity commanded this step
     */
    void setPrevCommandedVelocity(const Vector& local_velocity,
                                  const AngularVelocity& angular_velocity);

    RobotState state_;
    TbotsProto::Primitive current_primitive_;
    robot_constants::RobotConstants robot_constants_;

    std::optional<TrajectoryPath> trajectory_path_;
    std::optional<BangBangTrajectory1DAngular> angular_trajectory_;

    Duration time_since_linear_trajectory_creation_;
    Duration time_since_angular_trajectory_creation_;

    PositionController position_controller_;
    OrientationController orientation_controller_;

    // The velocities commanded on the previous step. Used to measure the commanded
    // (tick-to-tick) acceleration
    Vector prev_target_global_velocity_;
    AngularVelocity prev_target_angular_velocity_;

    // Forward-only mode: whether the robot is currently driving in reverse (facing the
    // opposite way to its travel direction). Persisted across steps to add hysteresis to
    // the forwards-vs-backwards decision so it doesn't chatter near perpendicular.
    bool forward_only_reversing_ = false;

    // Forward-only mode: once the robot is within this distance of its destination, it
    // stops slaving its heading to the travel direction and instead rotates to the
    // requested final orientation. [m]
    static constexpr double FORWARD_ONLY_FINAL_ROTATION_DISTANCE_M = 0.1;

    // Forward-only mode: pure-pursuit look-ahead time. The heading is aimed at the point
    // the planned path reaches this far in the future. Larger values are more damped (the
    // robot converges onto the path more gently, less weaving) but cut corners more;
    // smaller values track more tightly but can weave. [s]
    static constexpr double FORWARD_ONLY_LOOKAHEAD_TIME_S = 0.3;

    // Forward-only mode: the robot may drive in reverse when that needs a smaller turn.
    // To avoid chattering between facing forwards and backwards when the travel direction
    // is roughly perpendicular to the robot, only switch driving direction once the
    // alternative saves at least this much rotation. [rad]
    static constexpr double FORWARD_ONLY_REVERSE_HYSTERESIS_RAD = 0.35;  // ~20 deg

    // Forward-only mode: proportional gain [1/s] for the controller that rotates the
    // robot toward its target orientation (its pure-pursuit heading while moving, or the
    // requested final orientation near the destination). Higher gain reduces heading lag
    // on curves and corrects errors faster. The pure-pursuit look-ahead provides the
    // damping, so this is the knob for responsiveness: raise it for snappier turns, but
    // if the robot weaves across the path, increase FORWARD_ONLY_LOOKAHEAD_TIME_S or
    // lower this gain.
    static constexpr double FORWARD_ONLY_HEADING_KP = 8.0;

    // Forward-only mode: if the robot is within this angle of its target orientation,
    // stop commanding angular velocity so it settles instead of jittering on sensor
    // noise. [rad]
    static constexpr double FORWARD_ONLY_HEADING_DEADBAND_RAD = 0.017;  // ~1 deg

    // Estimated delay between a vision frame to AI processing to robot executing
    static constexpr double VISION_TO_ROBOT_DELAY_S = 0.03;

    // The distance away from the destination at which we start dampening the velocity
    // to avoid jittering around the destination.
    static constexpr double MAX_DAMPENING_VELOCITY_DISTANCE_M = 0.05;

    // If distance between current linear trajectory destination and new one is larger
    // than this, we change trajectories.
    static constexpr double LINEAR_DESTINATION_THRESHOLD_METERS   = 0.03;
    static constexpr double ANGULAR_DESTINATION_THRESHOLD_DEGREES = 4;
};
