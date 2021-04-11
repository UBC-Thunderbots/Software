#pragma once

#include <stdbool.h>

#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"

// The maximum size of the array containing trajectory elements. Assuming the
// longest possible path is 9 meters with 1mm segments
#define TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS 9000

typedef enum TrajectoryPlannerGenerationStatus
{
    OK,
    FINAL_VELOCITY_TOO_HIGH,
    INITIAL_VELOCITY_TOO_HIGH,
    // INTERPOLATION_ELEMENT_MAXED_OUT is returned when the constant period interpolation
    // function uses up all the available array space provided to it. This can happen
    // because constant period trajectories do not necessarily have the same
    // number of elements as their constant parameterization counterparts
    INTERPOLATION_ELEMENT_MAXED_OUT,
} TrajectoryPlannerGenerationStatus_t;

/*
 * NOTE: constant period means that the duration of time between each point
 * on the trajectory is CONSTANT. This is different from a constant parameterization
 * trajectory which has segments defined at constant polynomial evaluation intervals.
 * Constant arc-length trajectories (not used here) are trajectories where the distance
 * travelled in each segment in constant.
 */

typedef struct FirmwareRobotPathParameters
{
    /*
     * NOTE: Units must be consistent across the linear and angular domains. For example,
     * if the linear 'path' is represented in inits of millimeters the linear velocity
     * must be in mm/s and the acceleration in mm/s^2. The same goes for the angular
     * domain. If the angular path is specified in radians the angular velocity must be in
     * rad/s.
     */

    // The 2D polynomial representation of the path to be followed
    Polynomial2dOrder3_t path;
    // The 1D polynomial representation of the orientation to be followed
    Polynomial1dOrder3_t orientation_profile;
    // The path parameterization value indicating the beginning of the
    // considered path and orientation profile
    float t_start;
    // The path parameterization value indicating the end of the considered
    // path and orientation profile
    float t_end;
    // The number of elements to discretize the trajectory into.
    //  Must be greater than 2 elements.
    //  THE NUMBER OF SEGMENTS MUST BE UNDER
    //  TRAJECTORY_PLANNER_MAX_NUMBER_SEGMENTS
    unsigned int num_elements;
    // The maximum acceleration allowed at any
    // point along the trajectory. This factor limits the maximum delta-velocity
    // and also the max speed around curves due to centripetal acceleration [m/s^2]
    float max_allowable_linear_acceleration;
    // The maximum allowable linear speed allowable at any point on the
    // the trajectory
    float max_allowable_linear_speed;
    // The maximum allowable angular acceleration at any point along the orientation
    // profile
    float max_allowable_angular_acceleration;
    // The maximum allowable angular speed at any point along the orientation profile
    float max_allowable_angular_speed;
    // The initial speed at the start of the trajectory [m/s]
    float initial_linear_speed;
    // The final speed at the end of the trajectory [m/s]
    float final_linear_speed;
} FirmwareRobotPathParameters_t;

typedef struct PositionTrajectory
{
    float x_position[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_position[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float orientation[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float linear_speed[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_speed[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
} PositionTrajectory_t;

typedef struct VelocityTrajectory
{
    float x_velocity[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float y_velocity[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float angular_velocity[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
    float time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS];
} VelocityTrajectory_t;

/**
 * Function generates a constant parameterization position trajectory (each node is
 * defined by X,Y,Theta coordinates. Returns a planned trajectory with the list of
 * guarantees based on the assumptions below
 *
 *  Key assumptions & guarantees of this planner are:
 *  - Trajectories are time-optimal assuming INFINITE JERK capability of the robot
 *
 *  - No linear speed on the trajectory is larger than the 'max_allowable_linear_speed'
 * parameter'
 *  - No angular speed on the trajectory is larger than the 'max_allowable_angular_speed'
 * parameter
 *
 *  - No acceleration value between points on the trajectory can be larger than the
 * 'max_allowable_linear_acceleration' or 'max_allowable_angular_acceleration' input
 * parameter
 *
 *  - Assuming the grip-limit of the robot IS THE SAME AS THE MAX ACCELERATION then at no
 *    point on the path can the sum of centripetal and acceleration force be greater than
 *    'max_allowable_linear_acceleration'
 *
 *  PositionTrajectory generation is done by assuming constant acceleration capability.
 *      - The generator will assume maximum acceleration for the robot between each
 * segment on the path. If using max acceleration breaches either the speed limit of the
 * robot, or the centripetal acceleration limit defined by the curvature and robot speed -
 * the planner will assume the max acceleration that will remain below this limit.
 *
 *     - To ensure the robot is capable of decelerating to react to any sudden changes in
 *       the path, the trajectory is checked for backwards continuity
 *
 * NOTE: The following function is based on an algorithm described in page 18 of
 * http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
 *
 * @pre path_parameters members have been completely initialized and have values
 *
 * @param path_parameters The path parameters that define the physical limitations
 * and profile of the trajectory.
 *
 * @param position_trajectory [out] The trajectory data struct to be modified to contain
 * the position trajectory. path_parameters.num_elements number of elements will be
 * assigned to each of the trajectory member array.
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
    FirmwareRobotPathParameters_t path_parameters,
    PositionTrajectory_t *position_trajectory);

/**
 * Returns a constant period (time) trajectory based on an input trajectory
 *
 * This function is intended to take a variable period trajectory (which is easier to
 * calculate) then interpolate a constant period trajectory. This is valuable for discrete
 * time motion controllers that operate only at constant delta-time intervals
 *
 * NOTE: This function does not make any modifications to the dynamics of the trajectory.
 * It will output a physically invalid trajectory if the input trajectory is invalid
 *
 * @param variable_period_trajectory [in] The variable-period trajectory that will be used
 * to generate the constant-period trajectory.
 *
 * @param interpolation_period The interpolation period of the constant-period
 * trajectory. This parameter defines the length of time between successive elements in
 * the trajectory. In seconds.
 *
 * @param num_elements [in/out] The number of elements in the trajectory. The num_elements
 * parameter will be modified to contain the num_elements of the new constant-period
 * trajectory generated.
 *
 * @param constant_period_trajectory [out] The generated constant-period trajectory that
 * will be linearly interpolated from a variable-period trajectory.
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(
    PositionTrajectory_t *variable_period_trajectory, float interpolation_period,
    unsigned int *num_elements, PositionTrajectory_t *constant_period_trajectory);

/**
 * Function generates a constant period trajectory from the path parameters
 * specified in the constant parameterization trajectory. The function creates a constant
 * parameterization trajectory then uses linear interpolation to calculate the equivalent
 * constant period trajectory
 *
 * @param interpolation_period The duration of time between successive trajectory
 * elements. In seconds.
 *
 * @param path_parameters [in/out] The path parameters that define the trajectory. The
 * input path_parameters. num_elements defines the number of constant-parameterization
 * elements, and the returned path_parameters.num_elements defines the number of
 * constant-period elements in the trajectory,
 *
 * @param constant_period_trajectory [out] A constant period trajectory
 * generated by the specified path parameters.
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantPeriodPositionTrajectory(
    float interpolation_period, FirmwareRobotPathParameters_t *path_parameters,
    PositionTrajectory_t *constant_period_trajectory);

/**
 * Function generates a velocity trajectory composed of X/Y/angular velocities and their
 * time dependence.
 *
 * @param position_trajectory [in] A completely defined position trajectory.
 *
 * @param num_elements The number of elements in the trajectory.
 *
 * @param velocity_trajectory [out] A completely defined velocity trajectory that has
 * equally time-spaced elements. The velocity trajectory has the name number of elements
 * as the input position trajectory.
 */
void app_trajectory_planner_generateVelocityTrajectory(
    PositionTrajectory_t *position_trajectory, unsigned int num_elements,
    VelocityTrajectory_t *velocity_trajectory);
