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

    /**
     * Decide whether we should stop following the linear trajectory we're currently
     * following and start following the newly received one instead.
     *
     * We prefer to keep following the existing trajectory (so the controllers can
     * correct the robot back onto the planned path) and only switch when the new
     * trajectory's path deviates meaningfully from the one we're following (measured by
     * the Hausdorff distance between the two paths). We don't switch just because we're
     * behind/ahead, since we follow the trajectory by position (nearest point) rather
     * than by a wall clock.
     *
     * @param new_trajectory The newly received linear trajectory
     * @return true if we should start following new_trajectory
     */
    bool shouldFollowNewLinearTrajectory(const TrajectoryPath& new_trajectory) const;

    /**
     * Decide whether we should stop following the angular trajectory we're currently
     * following and start following the newly received one instead. Analogous to
     * shouldFollowNewLinearTrajectory but in orientation space.
     *
     * @param new_trajectory The newly received angular trajectory
     * @return true if we should start following new_trajectory
     */
    bool shouldFollowNewAngularTrajectory(
        const BangBangTrajectory1DAngular& new_trajectory) const;

    /**
     * Start following the given linear trajectory, resetting the position controller.
     * If we were already following a trajectory, set up a short blend so the commanded
     * velocity setpoint transitions smoothly into the new one.
     *
     * @param new_trajectory The linear trajectory to start following
     */
    void startFollowingNewLinearTrajectory(const TrajectoryPath& new_trajectory);

    /**
     * Start following the given angular trajectory, resetting the orientation
     * controller, and setting up a short blend as above.
     *
     * @param new_trajectory The angular trajectory to start following
     */
    void startFollowingNewAngularTrajectory(
        const BangBangTrajectory1DAngular& new_trajectory);

    /**
     * Find the time at which to sample the trajectory this step: the time of the point on
     * the trajectory nearest to the given position, plus a small look-ahead (clamped to
     * the trajectory's end). Sampling by nearest point anchors trajectory-following to
     * the robot's actual progress rather than to a wall clock.
     *
     * @param trajectory The trajectory being followed
     * @param position The robot's actual position
     * @return The time, in seconds, at which to sample the trajectory
     */
    double nearestTrajectorySampleTime(const TrajectoryPath& trajectory,
                                       const Point& position) const;

    /**
     * Orientation-space analogue of nearestTrajectorySampleTime.
     *
     * @param trajectory The angular trajectory being followed
     * @param orientation The robot's actual orientation
     * @return The time, in seconds, at which to sample the angular trajectory
     */
    double nearestAngularTrajectorySampleTime(
        const BangBangTrajectory1DAngular& trajectory, const Angle& orientation) const;

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

    // The trajectory we just switched away from, retained only for the duration of the
    // blend window so we can crossfade its velocity setpoint into the new trajectory's.
    // Reset (cleared) once the blend completes.
    std::optional<TrajectoryPath> prev_trajectory_path_;
    std::optional<BangBangTrajectory1DAngular> prev_angular_trajectory_;

    // Time remaining in the old->new trajectory blend. Zero when not blending.
    Duration linear_blend_remaining_;
    Duration angular_blend_remaining_;

    PositionController position_controller_;
    OrientationController orientation_controller_;

    // The velocities commanded on the previous step. Used to measure the commanded
    // (tick-to-tick) acceleration
    Vector prev_target_global_velocity_;
    AngularVelocity prev_target_angular_velocity_;

    // The distance away from the destination at which we start dampening the velocity
    // to avoid jittering around the destination.
    static constexpr double MAX_DAMPENING_VELOCITY_DISTANCE_M = 0.05;

    // How far ahead (in trajectory time) of the point nearest the robot we sample the
    // trajectory. This keeps the target leading the robot so it makes forward progress
    // along the path (without it, starting from rest the planned velocity at the nearest
    // point is ~0 and the robot would never start moving).
    static constexpr double TRAJECTORY_LOOKAHEAD_TIME_S = 0.05;

    // If the Hausdorff distance between the path of the trajectory we're following and
    // the path of a newly received trajectory exceeds this, the paths have deviated
    // enough that we switch to following the new trajectory.
    static constexpr double LINEAR_HAUSDORFF_THRESHOLD_M = 0.01;

    // If the new angular trajectory's final orientation differs from the current one's
    // by more than this, we switch to the new angular trajectory.
    static constexpr double ANGULAR_DESTINATION_THRESHOLD_DEGREES = 4;

    // Duration over which we blend the velocity setpoint from the old trajectory into
    // the new one after switching, so the setpoint doesn't change abruptly. Kept short
    // intentionally - it only smooths the switch, it shouldn't noticeably delay
    // tracking the new trajectory.
    static constexpr double TRAJECTORY_BLEND_DURATION_S = 0.04;
};
