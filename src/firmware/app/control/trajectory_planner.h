#pragma once

#include <stdbool.h>

#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"

#define TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS                                              \
    9000  // The maximum size of the array containing trajectory elements. Assuming the
          // longest possible path is 9 meters with 1mm segments

typedef struct FirmwareRobotPathParameters
{
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
    // The number of segments to discretize the trajectory into.
    //  Must be greater than 2 segments.
    //  THE NUMBER OF SEGMENTS MUST BE UNDER
    //  TRAJECTORY_PLANNER_MAX_NUMBER_SEGMENTS
    unsigned int num_segments;
    // The maximum acceleration allowed at any
    //  point along the trajectory. This factor limits the maximum delta-velocity
    // and
    //  also the max speed around curves due to centripetal acceleration [m/s^2]
    float max_allowable_linear_acceleration;
    // The maximum allowable angular acceleration allowable at any point on the
    // orientation profile The maximum speed allowable at any point along the trajectory
    float max_allowable_linear_speed;
    // The maximum allowable angular speed at any point along the orientation profile
    float max_allowable_angular_acceleration;
    // The maximum allowable angular speed at any point along the orientation profile
    float max_allowable_angular_speed;
    // The initial speed at the start of the trajectory [m/s]
    float initial_linear_speed;
    // The final speed at the end of the trajectory [m/s]
    float final_linear_speed;


} FirmwareRobotPathParameters_t;

// Struct that defines a single point on a trajectory
// Includes the Position and Time data corresponding to that point
typedef struct PositionTrajectoryElement
{
    Vector2d_t position;
    float linear_speed;
    float angular_speed;
    float orientation;
    float time;
} PositionTrajectoryElement_t;

typedef struct PositionTrajectory
{
    PositionTrajectoryElement_t* trajectory_elements;
    FirmwareRobotPathParameters_t path_parameters;
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

typedef struct TrajectorySegment
{
    float linear_segment_length;
    float angular_segment_length;
} TrajectorySegment_t;

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
 * Builds the forwards continuous linear velocity profile for the given parameters.
 * NOTE: The velocity profile returned by this function is NOT reverse continuous and does
 * not guarantee a feasible path to follow.
 *
 * NOTE: The velocity_profile will contain the same number of elements as the input
 * max_allowable_linear_speed[] array
 *
 * @pre The velocity_profile array is pre-allocated to contain up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS elements
 *
 * @param trajectory [in/out] The trajectory that contains the path parameters and will be
 * modified max_allowable_speed_profile input array
 *
 * @param max_allowable_speed_profile [in] The pre-allocated array that will contain all
 * of the maximum allowable velocity values for each point on the curve
 *
 * @param segment_lengths [in] The length of each corresponding segment [meters]
 *
 * @return The generation status. Can be OK or an error message
 * */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_createForwardsContinuousLinearSpeedProfile(
    PositionTrajectory_t* trajectory, float* max_allowable_speed_profile,
    TrajectorySegment_t* segment_lengths);

/**
 * Builds the forwards continuous linear velocity profile for the given parameters.
 * NOTE: The velocity profile returned by this function is NOT reverse continuous and does
 * not guarantee a feasible path to follow.
 *
 * NOTE: The velocity_profile will contain the same number of elements as the input
 * max_allowable_linear_speed[] array
 *
 * @pre The velocity_profile array is pre-allocated to contain up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS elements
 *
 * @param trajectory [in/out] The trajectory that contains the path parameters and will be
 * modified max_allowable_speed_profile input array
 *
 * @param segment_lengths [in] The length of each corresponding segment [meters]
 *
 * @return The generation status. Can be OK or an error message
 * */
void app_trajectory_planner_createForwardsContinuousAngularSpeedProfile(
    PositionTrajectory_t* trajectory, TrajectorySegment_t* segment_lengths);

/**
 * Function modifies an existing trajectory speed profile to be backwards continuous -
 * meaning that it is possible to deccelerate fast enough to reach each point on the
 * trajectory
 *
 * @pre trajectory has initialized path parameters
 *
 * @pre trajectory has pre-allocated arrays up to TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS
 *
 * @param trajectory [in/out] The trajectory to be have it's element speed profiles
 * modified to be backwards continuous
 *
 * @param segment_lengths [in] The length of each segment in the trajectory
 *
 * @return Trajectory generation status. Can be OK or error message
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_modifyTrajectoryToBackwardsContinuous(
    PositionTrajectory_t* trajectory, TrajectorySegment_t* segment_lengths);

/**
 * Function uses the speed profiles specified by the trajectory to calculate the
 * cumulative duration of time required to reach each point
 *
 * @pre arrays are pre-allocated up to TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS
 *
 * @pre trajectory has existing speed profiles for linear and angular motion
 *
 * @param trajectory [in/out] Trajectory containing the speed profiles to be used to
 * calculate the time profile of the trajectory
 *
 * @param segment_lengths [in] The length of each segment in the trajectory
 */
void app_trajectory_planner_generatePositionTrajectoryTimeProfile(
    PositionTrajectory_t* trajectory, TrajectorySegment_t* segment_lengths);

/**
 * Returns the maximum velocity profile for a given curve based on curvature and maximum
 * allowable acceleration
 *
 * @pre max_allowable_speed_profile is pre-allocated up at least the limit specified by
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param path [in] The polynomial that defines the path of the trajectory
 *
 * @param num_elements [in] The number of elements in the trajectory
 *
 * @param t_start [in] The starting parameterization value of the path
 *
 * @param t_end [in] The final parameterization value of the path
 *
 * @param max_allowable_acceleration [in] The max allowable acceleration at any discrete
 * point on the trajectory
 *
 * @param max_allowable_speed_profile [out] The pre-allocated array that will contain all
 * of the maximum allowable velocity values for each point on the curve
 *
 */
void app_trajectory_planner_getMaxAllowableSpeedProfile(
    Polynomial2dOrder3_t path, const unsigned int num_elements, const float t_start,
    const float t_end, const float max_allowable_acceleration,
    float* max_allowable_speed_profile);


/**
 * Function rebalances the specified linear or angular segment of a trajectory to have the
 * specified duration. The final speed at the end of the segment will be modified to reach
 * the specified value
 *
 * @pre segment_lengths has been allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @pre trajectory has pre allocated element array up to
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param trajectory [in/out] The trajectory to be modified
 *
 * @param segment_lengths [in] The length of each segment in the trajectory
 *
 * @param desired_segment_duration [in] The desited segment duration in seconds
 *
 * @param trajectory_index [in] The index of end-point of the segment to be modified
 *
 * @param rebalance_angular [in] Boolean to select if the angular or linear segment is to
 * be modified. rebalance_angular == true means that the angular segment will be modified
 * @return
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_rebalanceAngularAndLinearTrajectorySegmentsForEquivalentDuration(
    PositionTrajectory_t* trajectory, TrajectorySegment_t* segment_lengths,
    const float desired_segment_duration, unsigned int trajectory_index,
    bool rebalance_angular);

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

/**
 * Returns a planned trajectory with the list of guarantees based on the assumptions below
 *
 *  Key assumptions & guarantees of this planner are:
 *  - Trajectories are time-optimal assuming INFINITE JERK capability of the robot
 *
 *  - No speed on the trajectory is larger than the 'max_allowable_linear_speed' parameter
 *
 *  - No acceleration value between points on the trajectory can be larger than the
 * 'max_allowable_linear_acceleration' input parameter
 *
 *  - Assuming the grip-limit of the robot IS THE SAME AS THE MAX ACCELERATION then at no
 * point on the path can the sum of centripetal and acceleration force be greater than
 * 'max_allowable_linear_acceleration'
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
 * @pre path_parameters.max_allowable_linear_acceleration > 0
 * @pre path_parameters.max_allowable_linear_speed > 0
 * @pre path_parameters.init_speed >= 0
 * @pre path_parameters.final_linear_speed >= 0
 * @pre The trajectory.trajectory_elements[] array is pre-allocated to handle up to the
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS limit
 *
 * @param trajectory [in/out] The trajectory with pre-initialized path parameters. Will be
modified in-place
 *
 * @return The generation status. Can be OK or error message.
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantParameterizationPositionTrajectory(
    PositionTrajectory_t* trajectory);

/**
 * Function generates a constant interpolation period trajectory from the path parameters
 * specified in the constant parameterization trajectory. The function creates a constant
 * parameterization trajectory then uses linear interpolation to calculate the equivalent
 * constant interpolation period trajectory
 *
 * @pre The elements the trajectory are pre-allocated up to
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @pre The path parameters of the variable_time_trajectory are all initialized
 *
 * @param path_parameters [in] The path parameters for the trajectory
 *
 * @param constant_period_trajectory [in/out] Pre-allocated trajectory that will be
 * modified to represent a constant interpolation trajectory
 *
 * @param interpolation_period [in] The interpolation period desired of the constant
 * interpolation period trajectory
 *
 * @return Generation status, this can be OK or an error message
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantInterpolationPeriodPositionTrajectory(
    FirmwareRobotPathParameters_t path_parameters,
    PositionTrajectory_t* constant_period_trajectory, const float interpolation_period);

/**
 * This function calculates the robot state at every point along the trajectory (X/Y amd
 * orientation) along with the physical length of these segments (meters, radians)
 *
 * @pre All arrays in the Trajectory must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS
 *
 * @param trajectory [in] The trajectory with pre-allocated arrays to contain trajectory
 * data
 * @param velocity_trajectory [out] The velocity trajectory that corresponds to the
 * time-optimal velocity to follow a specified path
 */
void app_trajectory_planner_generateStatesAndReturnSegmentLengths(
    PositionTrajectory_t* trajectory,
    TrajectorySegment_t trajectory_segments[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function generates a constant interpolation period trajectory based on the input path
 * parameters and interpolation period
 *
 * @pre path parameter variables must all be initialized
 *
 * @pre velocity trajectory element array must be pre-allocated up to
 * TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS
 *
 * @pre interpolation_period must be greater than zero
 *
 * @param path_parameters [in] The path parameters that define the trajectory
 *
 * @param velocity_trajectory [out] The constant interpolation period velocity trajectory
 * generated
 *
 * @param interpolation_period ; The interpolation period of the trajectory in seconds
 *
 * @return Generation status. Can be OK or an error message.
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantInterpolationVelocityTrajectory(
    FirmwareRobotPathParameters_t path_parameters,
    VelocityTrajectory_t* velocity_trajectory, const float interpolation_period);