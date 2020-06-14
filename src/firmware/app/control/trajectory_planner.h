#pragma once

#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"

#define TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS                                              \
    9000  // The maximum size of the array containing trajectory elements. Assuming the
          // longest possible path is 9 meters with 1mm segments

typedef struct FirmwareRobotPathParameters
{
    // The 2D polynomial representation of the path to be followed
    Polynomial2dOrder3_t path;
    // The path parameterization value indicating the beginning of the
    // considered path
    float t_start;
    // The path parameterization value indicating the end of the considered
    // path
    float t_end;
    // The number of segments to discretize the trajectory into.
    //  Must be greater than 2 segments.
    //  THE NUMBER OF SEGMENTS MUST BE UNDER
    //  TRAJECTORY_PLANNER_MAX_NUMBER_SEGMENTS
    unsigned int num_segments;
    // The maximum acceleration allowed at any
    //  point along the trajectory. This factor limits the maximum delta-velocity
    // and
    //  also the max speed around curves due to centripetal acceleration [m/s^2]
    float max_allowable_acceleration;
    // The maximum speed allowable at any point along the trajectory```
    float max_allowable_speed;
    // The initial speed at the start of the trajectory [m/s]
    float initial_speed;
    // The final speed at the end of the trajectory [m/s]
    float final_speed;
} FirmwareRobotPathParameters_t;

// Struct that defines a single point on a trajectory
// Includes the Position and Time data corresponding to that point
typedef struct PositionTrajectoryElement
{
    Vector2d_t position;
    float time;
} PositionTrajectoryElement_t;

typedef struct PositionTrajectory
{
    PositionTrajectoryElement_t* trajectory_elements;
    FirmwareRobotPathParameters_t path_parameters;
    float* speed_profile;
} PositionTrajectory_t;

// Struct that defines a single point on a velocity trajectory
// Includes the velocity and Time data corresponding to that point
typedef struct VelocityTrajectoryElement
{
    Vector2d_t linear_velocity;
    float angular_velocity;
    float time;
} VelocityTrajectoryElement_t;

typedef struct VelocityTrajectory
{
    VelocityTrajectoryElement_t* trajectory_elements;
    FirmwareRobotPathParameters_t path_parameters;
} VelocityTrajectory_t;

typedef enum TrajectoryPlannerGenerationStatus
{
    OK,
    FINAL_VELOCITY_TOO_HIGH,
    INITIAL_VELOCITY_TOO_HIGH,

    // INTERPOLATION_ELEMENT_MAXED_OUT is returned when the constant time interpolation
    // function uses up all the avaiable array space provided to it. This can happen
    // because constant interpolation period trajectories do not necessarily have the same
    // number of elements as their constant arc-length counterparts
    INTERPOLATION_ELEMENT_MAXED_OUT,
} TrajectoryPlannerGenerationStatus_t;

/**
 * Returns a planned trajectory with the list of guarantees based on the assumptions below
 *
 *  Key assumptions & guarantees of this planner are:
 *  - Trajectories are time-optimal assuming INFINITE JERK capability of the robot
 *
 *  - No speed on the trajectory is larger than the 'max_allowable_speed' parameter
 *
 *  - No acceleration value between points on the trajectory can be larger than the
 * 'max_allowable_acceleration' input parameter
 *
 *  - Assuming the grip-limit of the robot IS THE SAME AS THE MAX ACCELERATION then at no
 * point on the path can the sum of centripetal and acceleration force be greater than
 * 'max_allowable_acceleration'
 *
 *  PositionTrajectory generation is done by assuming constant acceleration capability.
 *      - The generator will assume maximum acceleration for the robot between each
 * segment on the path. If using max acceleration breaches either the speed limit of the
 * robot, or the centripetal acceleration limit defined by the curvature and robot speed -
 * the planner will assume the max acceleration that will remain bellow this limit.
 *
 *     - To ensure the robot is capable of deccelerating to react to any sudden changes in
 * the path, the trajectory is checked for backwards continuity
 *
 * NOTE: The following function is based on an algorithm described in page 18 of
 * http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
 *
 * @pre path_parameters.num_segments > 2 and num_segments <=
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 * @pre path_parameters.max_allowable_acceleration > 0
 * @pre path_parameters.max_allowable_speed > 0
 * @pre path_parameters.init_speed >= 0
 * @pre path_parameters.final_speed >= 0
 * @pre The trajectory.trajectory_elements[] array is pre-allocated to handle up to the
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS limit
 *
 * @param path_parameters [in] The data structure including important path parameters as
 * defined by the FirmwareRobotPathParameters struct
 * @param trajectory [out] The planned trajectory that follows robot dynamics limitations
 * with the appropriate guarantees and assumptions outlines above
 *
 * @return The outcome of the trajectory generation. Returns OK if trajectory is valid.
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantArcLengthPositionTrajectory(
    PositionTrajectory_t* trajectory);

/**
 * Returns a constant interpolation period (time) trajectory based on an input trajectory
 *
 * This function is intended to take a variable time trajectory (which is easier to
 * calculate) then interpolate a constant period trajectory. This is valuable for discrete
 * time motion controllers that operate only at constant delta-time intervals
 *
 * NOTE: This function does not modify the real trajectory. It will output a physically
 * invalid trajectory if the input trajectory is invalid
 *
 * @pre constant_period_trajectory is pre-allocated up the the limit specified by
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param constant_period_trajectory [out] The array to be filled with the constant time
 * interpolation period equivalent of the input trajectory
 * @param variable_time_trajectory [in] The valid trajectory used as
 * reference for a constant interpolation period trajectory
 * @param interpolation_period [in] The constant change in time [s] that corresponds to
 * each trajectory segment.
 *
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
    PositionTrajectory_t* constant_period_trajectory,
    PositionTrajectory_t* variable_time_trajectory, const float interpolation_period);

/**
 * Generates the X/Y points for each position on the constant arc length trajectory
 *
 * @pre traj_elements [out] is pre-allocated up the the limit specified by
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS. Will be modified to contain the X/Y points of the
 * trajectory
 *
 * @param path [in] The polynomial that defines the path of the trajectory
 *
 * @param num_elements [in] The number of elements in the trajectory
 *
 * @param arc_length_parameterization [in] The arc length parameterization of the path
 * polynomial
 *
 * @param arc_segment_length [in] The length of each segment in the trajectory
 *
 */
static void app_trajectory_planner_generateConstArclengthTrajectoryPositions(
    PositionTrajectoryElement_t traj_elements[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    Polynomial2dOrder3_t path, const unsigned int num_elements,
    ArcLengthParametrization_t arc_length_parameterization,
    const float arc_segment_length);

/**
 * Returns the maximum velocity profile for a given curve based on curvature and maximum
 * allowable acceleration
 *
 * @pre max_allowable_speed_profile is pre-allocated up at least the limit specified by
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param max_allowable_speed_profile [out] The pre-allocated array that will contain all
 * of the maximum allowable velocity values for each point on the curve
 *
 * @param path [in] The polynomial that defines the path of the trajectory
 *
 * @param num_elements [in] The number of elements in the trajectory
 *
 * @param arc_length_parameterization [in] The arc length parameterization of the path
 * polynomial
 *
 * @param arc_segment_length [in] The length of each segment in the trajectory
 *
 * @param max_allowable_acceleration [in] The max allowable acceleration at any discrete
 * point on the trajectory
 *
 */
void app_trajectory_planner_getMaxAllowableSpeedProfile(
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    Polynomial2dOrder3_t path, const unsigned int num_elements,
    ArcLengthParametrization_t arc_length_parameterization,
    const float arc_segment_length, const float max_allowable_acceleration);

/**
 * Builds the forwards continuous velocity profile for the given parameters.
 * NOTE: The velocity profile returned by this function is NOT reverse continuous and does
 * not guarantee a feasible path to follow.
 *
 * NOTE: The velocity_profile will contain the same number of elements as the input
 * max_allowable_speed[] array
 *
 * @pre The velocity_profile array is pre-allocated to contain up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS elements
 *
 * @param num_segments [in] The number of segments(elements) in the
 * max_allowable_speed_profile input array
 * @param velocity_profile [out] The array that the forwards continuous array will be
 * copied into
 * @param max_allowable_speed_profile [in] The pre-allocated array that will contain all
 * of the maximum allowable velocity values for each point on the curve
 * @param arc_segment_length [in] The length of each arc length segment in meters
 * @param max_allowable_acceleration [in] The max allowable acceleration at any point in
 * m/s^2
 * @param max_allowable_speed [in] The max allowable speed at any point in m/s
 *
 * */
void app_trajectory_planner_generateForwardsContinuousVelocityProfile(
    const unsigned int num_segments,
    float velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float arc_segment_length, const float max_allowable_acceleration,
    const float max_allowable_speed);

/**
 * Modifies a forwards continuous velocity profile to also be backwards continuous based
 * on the input parameters.
 *
 * NOTE: This function does not break forwards continuity of the trajectory
 *
 * @pre The input velocity_profile is required to be FORWARDS CONTINUOUS before it is used
 * as an input to this function
 * @pre forwards_continuous_velocity_profile is pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param num_segments [in] The number of segments(elements) in the velocity_profile array
 * @param forwards_continuous_velocity_profile [in/out] The forwards continuous velocity
 * profile to ebe modified in-place into a profile that is also backwards continuous
 * @param arc_segment_length [in] The arc segment length of each segment in the path the
 * profile is being generated for in meters
 * @param max_allowable_acceleration [in] The max allowable acceleration at any point on
 * the velocity profile
 */
void app_trajectory_planner_generateBackwardsContinuousVelocityProfile(
    const unsigned int num_segments,
    float forwards_continuous_velocity_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    const float arc_segment_length, const float max_allowable_acceleration);

/**
 *  Adds a time profile to an existing position profile for the given input parameters
 *
 *  @pre traj_elements must be pre-allocated and contain all of the position data of the
 * trajectory
 * @pre traj_elements & velocity_profile are pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param traj_elements [in/out] The trajectory element array that will be modified to
 * contain the time profile
 * @param num_segments [in] The number of segments(elements) in traj_elements and
 * velocity_profile
 * @param arc_segment_length  [in] The arc length of each path segment
 * @param velocity_profile [in] The forwards and backwards continuous velocity profile of
 * the trajectory
 */
void static app_trajectory_planner_generatePositionTrajectoryTimeProfile(
    PositionTrajectoryElement_t* traj_elements, const float num_segments,
    const float arc_segment_length, float* velocity_profile);

/***
 *  This function takes in a forwards trajectory annd modifies it in place to become a
 * backwards trajectory NOTE: This function exists to avoid issues outlined in #1322
 *
 *  TODO: Remove when #1322 is merged.
 *
 * @param forwards_trajectory This is the trajectory that will be modified in place to
 * become a reverse trajectory
 */
void static app_trajectory_planner_reversePositionTrajectoryDirection(
    PositionTrajectory_t* forwards_trajectory);

/**
 * This function generates a velocity trajectory that corresponds to the time-optimal
 * velocity to follow a specified path. This profile is based on the input position
 * trajectory
 *
 * @param position_trajectory [in] The position trajectory to build the velocity
 * trajectory from
 * @param velocity_trajectory [out] The velocity trajectory that corresponds to the
 * time-optimal velocity to follow a specified path
 */
void app_trajectory_planner_generateVelocityTrajectory(
    PositionTrajectory_t* position_trajectory, VelocityTrajectory_t* velocity_trajectory);
