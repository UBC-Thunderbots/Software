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
*   t_start - The path parameter value indicating the beginning of the
*       considered path
*   t_end - The path parameter value indicating the end of the considered
*       path
*
*   num_segments - The number of segments to discretize the trajectory into.
*       Must be greater than 2 segments.
*       THE NUMBER OF SEGMENTS MUST BE UNDER
*       TRAJECTORY_PLANNER_MAX_NUMBER_SEGMENTS
*
*   max_allowable_acceleration - The maximum acceleration allowed at any
*       point along the trajectory. This factor limits the maximum delta-velocity and also the
*       max speed around curves due to centripetal acceleration [m/s^2]
*
*   max_allowable_speed - The maximum speed allowable at any point along the
*
*   path [m/s] *initial_speed - The initial speed at the start of the trajectory [m/s]
*
*   final_speed - The final speed at the end of the trajectory [m/s]
 */
typedef struct FirmwareRobotPathParameters
{
    Polynomial2dOrder3_t path;
    float t_start;
    float t_end;
    unsigned int num_segments;
    float max_allowable_acceleration;
    float max_allowable_speed;
    float initial_speed;
    float final_speed;
} FirmwareRobotPathParameters_t;

/**
 * Returns a planned trajectory as a list of TrajectoryElement's.
 *
 * Key assumptions of this planner are:
 *  - Trajectories are time-optimal assuming INFINITE JERK capability of the robot
 *  - No speed on the trajectory is larger than the 'max_allowable_speed' parameter.
 *  - No acceleration value between points on the trajectory can be larger than the
 * 'max_allowable_acceleration' input parameter
 *  - Assuming the grip-limit of the robot IS THE SAME AS THE MAX ACCELERATION and at no
 * point on the path can the sum of centripital and acceleration force be greather than
 * 'max_allowable_acceleration'
 *
 *  Trajectory generation is done by assuming constant acceleration capability.
 *      - The generator will assume maximum acceleration for the robot between each
 * segment on the path. If using max acceleration breaches either the speed limit of the
 * robot, or the centripital acceleration limit defind by the curvature and robot speed -
 * the planner will assume the max acceleration that will remain bellow this limit.
 *      - To ensure the robot is capable of deccelerating to reach ant sudden changes in
 * the path, the trajectory is checked for backwards continuity
 *
 * @pre t_start < t_end
 * @pre num_segments > 2
 * @pre max_allowable_acceleration > 0
 * @pre max_allowable_speed > 0
 * @pre init_speed > 0
 * @pre final_speed > 0
 *
 *  @param path_parameters [in] The data structure including important path parameters
 * such as: *path - The 3D polynomial representing the geometric path
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
 * @pre variable_time_trajectory is a valid trajectory
 *
 * @param constant_period_trajectory [in] The array to be filled with the constant time
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
 * @param max_allowable_speed_profile [in] The pre-allocated array that will contain all
 * of the maximum allowable velocity values for each point on th curve
 *
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