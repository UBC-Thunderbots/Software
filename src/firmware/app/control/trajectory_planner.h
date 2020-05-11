#pragma once

#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"

#define TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS                                              \
    6000  // The maximum size of the array containing trajectory elements

// Struct that defines a single point on a trajectory
// Includes the Position,and Time data corresponding to that point
typedef struct TrajectoryElement
{
    Vector2d_t position;
    float time;
} TrajectoryElement_t;

typedef struct Trajectory
{
    TrajectoryElement_t* trajectory_elements;
    unsigned int num_elements;
} Trajectory_t;

/*



 *   final_speed - The final speed at the end of the trajectory [m/s]
 */
typedef struct FirmwareRobotPathParameters
{
    // The 2D polynomial representation of the path to be followed
    Polynomial2dOrder3_t path;
    // The path parameter value indicating the beginning of the
    // considered path
    float t_start;
    // The path parameter value indicating the end of the considered
    // path
    float t_end;
    // The number of segments to discretize the trajectory into.
    // *       Must be greater than 2 segments.
    // *       THE NUMBER OF SEGMENTS MUST BE UNDER
    // *       TRAJECTORY_PLANNER_MAX_NUMBER_SEGMENTS
    unsigned int num_segments;
    // The maximum acceleration allowed at any
    // *       point along the trajectory. This factor limits the maximum delta-velocity and
    // *       also the max speed around curves due to centripetal acceleration [m/s^2]
    float max_allowable_acceleration;
    // The maximum speed allowable at any point along the
    float max_allowable_speed;
    // The initial speed at the start of the trajectory [m/s]
    float initial_speed;
    // The final speed at the end of the trajectory [m/s]
    float final_speed;
} FirmwareRobotPathParameters_t;

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
 *  Trajectory generation is done by assuming constant acceleration capability.
 *      - The generator will assume maximum acceleration for the robot between each
 * segment on the path. If using max acceleration breaches either the speed limit of the
 * robot, or the centripetal acceleration limit defined by the curvature and robot speed -
 * the planner will assume the max acceleration that will remain bellow this limit.
 *
 *     - To ensure the robot is capable of deccelerating to reach ant sudden changes in
 * the path, the trajectory is checked for backwards continuity
 *
 * @pre path_parameters.num_segments > 2
 * @pre path_parameters.max_allowable_acceleration > 0
 * @pre path_parameters.max_allowable_speed > 0
 * @pre path_parameters.init_speed >= 0
 * @pre path_parameters.final_speed >= 0
 * @pre The trajectory.trajectory_elements[] array is pre-allocated ot handle up to the
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS limit
 *
 * @param path_parameters [in] The data structure including important path parameters as
 * defined by the FirmwareRobotPathParameters struct
 * @param trajectory [out] The planned trajectory that follows robot dynamics limitations
 * with the appropriate guarantees and assumptions outlines above
 */
void app_trajectory_planner_generate_constant_arc_length_segmentation(
    FirmwareRobotPathParameters_t path_parameters, Trajectory_t* trajectory);

/**
 * Returns a constant interpolation period (time) trajectory based on an input trajectory
 *
 * This function is intended to take a variable time trajectory (which is easier to
 * calculate) then interpolate a constant period trajectory. This is valuable for discrete
 * time motion controllers that operate only at constant delta-time intervals
 *
 * @pre variable_time_trajectory is a valid (obeys the physical limitation of the robot)
 * trajectory
 * @pre constant_period_trajectory is pre=allocated up the the limit specified by
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param constant_period_trajectory [out] The array to be filled with the constant time
 * interpolation period equivalent of the input trajectory
 * @param variable_time_trajectory [in] The valid trajectory used as
 * reference for a constant interpolation period trajectory
 * @param interpolation_period [in] The constant change in time that corresponds to each
 * trajectory segment
 *
 */
void app_trajectory_planner_interpolate_constant_time_trajectory_segmentation(
    Trajectory_t* constant_period_trajectory, Trajectory_t* variable_time_trajectory,
    const float interpolation_period);


/**
 * Returns a the total length of any arc specified by an arc parameterizaton
 *
 * @param arc_length_param [in] Arc length parameterization
 *
 * @return [out] The arc length in meters
 */
static float app_trajectory_planner_get_total_arcLength(
    ArcLengthParametrization_t arc_length_param);

/**
 * Returns the maximum velocity profile for a given curve based on curvature and maximum
 * allowable acceleration
 *
 * @param max_allowable_speed_profile is pre-allocated up the the limit specified by
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param max_allowable_speed_profile [out] The pre-allocated array that will contain all
 * of the maximum allowable velocity values for each point on th curve
 * @param traj_elements [in] Trajectory segments including X/Y position
 *
 * @param path [in] The polynomial that defines the path of the trajectory
 *
 * @param num_elements [in] The number of elements in the trajectory
 *
 * @param arc_length_param [in] The arc length parameterization of the path polynomial
 *
 * @param arc_segment_length [in] The length of each segment in the trajectory
 *
 * @param max_allowable_acceleration [in] The max allowable acceleration at any discrete
 * point on the trajectory
 *
 */
static void app_trajectory_planner_get_max_allowable_speed_profile(
    float* max_allowable_speed_profile, TrajectoryElement_t* traj_elements,
    Polynomial2dOrder3_t path, unsigned int num_elements,
    ArcLengthParametrization_t arc_length_param, float arc_segment_length,
    const float max_allowable_acceleration);

/**
 * Builds the forwards continuous velocity profile for the given parameters.
 * NOTE: The velocity profile returned by this function is NOT reverse continuous and does
 * not guarantee a feasible path to follow.
 *
 * NOTE: The velocity_profile wil contain the same number of real elements as the input
 * max_allowable_speed[] array
 *
 * @pre The velocity_profile array is pre-allocated to contain up to
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS elements
 *
 * @param num_segments [in] The number of segments(elements) in the
 * max_allowable_speed_profile input array
 * @param velocity_profile [out] The array that the forwards continuous array will be
 * copied into
 * @param max_allowable_speed_profile [in] The pre-allocated array that will contain all
 * of the maximum allowable velocity values for each point on th curve
 * @param arc_segment_length [in] The lenmgth of each arc length segment in meters
 * @param max_allowable_acceleration [in] The max allowable acceleration at any point in
 * m/s^2
 * @param max_allowable_speed [IN] The max allowable speed at any point in m/s
 *
 * */
void app_trajectory_planner_generate_forwards_continuous_velocity_profile(
    unsigned int num_segments, float* velocity_profile,
    float* max_allowable_speed_profile, const float arc_segment_length,
    const float max_allowable_acceleration, const float max_allowable_speed);

/**
 * Modifies a forwards continuous velocity profile to also be backwards continuous based
 * on the input parameters.
 *
 * @pre The input velocity_profile is required to be FORWARDS CONTINUOUS before it is used
 * as an input to this function
 *
 * @param num_segments [in] The number of segments(elements) in the velocity_profile array
 * @param forwards_continuous_velocity_profile The forwards continuous velocity profile to
 * ebe modified in-place into a profile that is also backwards continuous
 * @param arc_segment_length [in] The arc segment length of each segment in the path the
 * profile is being generated for in meters
 * @param max_allowable_acceleration [in] The max allowable acceleration at any point on
 * the velocity profile
 */
void app_trajectory_planner_generate_backwards_continuous_velocity_profile(
    unsigned int num_segments, float* forwards_continuous_velocity_profile,
    const float arc_segment_length, const float max_allowable_acceleration);

/**
 *  Adds a time profile to an existing position profile for the given input parameters
 *
 *  @pre traj_elements must be pre-allocated and contain all of the position data of the
 * trajectory
 *
 * @param traj_elements The trajectory element array that will be modified to contain the
 * time profile
 * @param num_segments [in/ The number of segments(elements) in traj_elements and
 * velocity_profile
 * @param arc_segment_length  [in] The arc length of each path segment
 * @param velocity_profile  The forwards and backwards continuous velocity profile of the
 * trajectory
 */
void app_trajectory_planner_generate_time_profile(TrajectoryElement_t* traj_elements,
                                                  const float num_segments,
                                                  const float arc_segment_length,
                                                  float* velocity_profile);