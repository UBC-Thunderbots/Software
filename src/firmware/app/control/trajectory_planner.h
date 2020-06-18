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

typedef enum TrajectoryPlannerGenerationStatus
{
    OK,
    FINAL_VELOCITY_TOO_HIGH,
    INITIAL_VELOCITY_TOO_HIGH,
    // INTERPOLATION_ELEMENT_MAXED_OUT is returned when the constant time interpolation
    // function uses up all the available array space provided to it. This can happen
    // because constant interpolation period trajectories do not necessarily have the same
    // number of elements as their constant parameterization counterparts
    INTERPOLATION_ELEMENT_MAXED_OUT,
} TrajectoryPlannerGenerationStatus_t;

/**
 * Function generates the segment lengths and the states at each node along the specified
 * constant-parameterization segments
 *
 * @pre All arrays must be pre-allocated up to TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param t_start [in] The starting parameterization value
 *
 * @param t_end  [in] The ending parameterization value
 *
 * @param poly  [in] The 1d polynomial representing the path along the trajectory
 *
 * @param num_elements [in] The number of elements (nodes) to be in the trajectory
 *
 * @param node_values [out] The evaluation of the polynomial at each value of the constant
 * parameterization
 *
 * @param segment_lengths [out] The length of segments between each node specified by the
 * constant parameterization
 *
 * Node1  |--segment length---| Node2
 *   ->   *-------------------*  <-
 */
void app_trajectory_planner_generateSegmentNodesAndLengths(float t_start, float t_end, Polynomial1dOrder3_t poly,
                                                           unsigned int num_elements, float node_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
                                                           float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function generates the length of segments in a 2d polynomial. It works as an wrapper
 * for the euclidean distance and values of 2d polynomials.
 *
 * @pre All arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param t_start [in] The starting value of polynomial parameterization
 *
 * @param t_end [in] The final value of polynomial parameterization
 *
 * @param poly [in] The polynomial representing the trajectory in space
 *
 * @param num_elements [in] The number of nodes(elements) of the generated trajectory
 *
 * @param x_values [out] The x value of each trajectory node
 *
 * @param y_values [out] The y value of each trajectory node
 *
 * @param segment_lengths [out] The length of each trajectory segment (euclidean of the XY
 * 2d polynomial)
 */
void app_trajectory_planner_generateLinearSegmentNodesAndLengths(float t_start,float t_end,
                                                                 Polynomial2dOrder3_t poly,
                                                                 unsigned int num_elements, float x_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
                                                                 float y_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

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
 * 'max_allowable_linear_acceleration' or 'max_allowable_angular_acceleration input
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
 * the planner will assume the max acceleration that will remain bellow this limit.
 *
 *     - To ensure the robot is capable of decelerating to react to any sudden changes in
 * the path, the trajectory is checked for backwards continuity
 *
 * NOTE: The following function is based on an algorithm described in page 18 of
 * http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
 *
 * @pre path_parameters members have been completely initialized and have values
 *
 * @param path_parameters [in] The path parameters that define the physical limitations
 * and profile of the trajectory.
 *
 * @param position_trajectory [out] The trajectory data struct to be modified to contain
 * the position trajectory
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantParameterizationPositionTrajectory(FirmwareRobotPathParameters_t path_parameters,
                                                                          PositionTrajectory_t *position_trajectory);

/**
 * Function that modifies an existing speed profile to be backwards continuous. This means
 * that it is possible to reach every speed node within the limits of deceleration
 *
 * @pre All arrays are pre-allocated up to at least TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param segment_lengths [in] The length of each segment in the trajectory
 *
 * @param max_allowable_acceleration [in] The maximum acceleration that can occur anywhere
 * on the trajectory
 *
 * @param initial_speed [in] The initial speed at the beggining of the trajectory
 *
 * @param num_segments [in] The number of nodes(elements) in the trajectory
 *
 * @param speeds [in/out] The existing speed profile to be modified in-place to be
 * backwards continuous
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_modifySpeedsToBackwardsContinuous(unsigned int num_segments, float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
                                                         float max_allowable_acceleration, float initial_speed,
                                                         float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function creates a forwards continuous speed profile based on the specified parameters.
 * The profile will never exceed the maximums speeds specified by the
 * max_allowable_speed_profile
 *
 * @pre All arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param segment_lengths [in] The length of each segment between data points on the speed
 * profile. In meters.
 *
 * @param max_allowable_speed_profile [in] An array that limits the max speed at each
 * point on the profile. In m/s.
 *
 * @param max_allowable_acceleration [in] The maximum allowable acceleration along any
 * segment in the profile In m/s^2.
 *
 * @param initial_speed [in] The initial speed at the first element of the profile. In
 * m/s.
 *
 * @param final_speed  [in] The final speed at the last element of the profile. In m/s.
 *
 * @param num_elements [in] The number of elements in the speed profile
 *
 * @param speeds [out] The pre-allocated array that will be modified to contain a forwards
 * continuous speed profile that obeys the initial and final speeds and the
 * max_allowable_speed_profile that limits the speed at each point on the trajectory. In
 * m/s.
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_createForwardsContinuousSpeedProfile(unsigned int num_elements, float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
                                                            float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
                                                            float max_allowable_acceleration, float initial_speed,
                                                            float final_speed, float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function generates the absolute maximum speed that can occur at any points along the
 * trajectory. This maximum speed is determined by the limit of grip (in acceleration)
 * along any curved path, along with imposed speed limits of the user.
 *
 * @pre All arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS
 *
 * @param path [in] The 2d polynomial that defines the path in 2d space.
 *
 * @param num_elements [in] The number of elements that will make up the trajectory
 *
 * @param t_start [in] The starting value of the parameterization
 *
 * @param t_end [in] The ending value of the parameterization
 *
 * @param max_allowable_acceleration [in] The maximum allowable acceleration along the
 * trajectory. In m/s^2.
 *
 * @param speed_cap [in] The maximum allowable speed at any point on the trajectory. This
 * value is defined by the user and this speed limit will not be exceeded even if possible
 * given the physical possibility. In m/s.
 *
 * @param max_allowable_speed_profile [out] The array that will be modified in-place to
 * contain the absolute maximum allowable speed profile. In m/s.
 */
void app_trajectory_planner_getMaximumSpeedProfile(
        Polynomial2dOrder3_t path, const unsigned int num_elements, const float t_start, const float t_end,
        const float max_allowable_acceleration, const float speed_cap,
        float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function generates the time profile for a speed profile with given segment lengths and
 * start and end velocities. This is done using a constant acceleration assumption over
 * the total displacement of each segment
 *
 * @pre All arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param segment_lengths [in] The length of each segment in meters. The segment length is
 * the distance travelled between successive speed elements.
 *
 * @param speeds [in] The speed profile defining the current speed at each element. In
 * m/s.
 *
 * @param num_elements [in] The number of elements in the speed profile.
 *
 * @param trajectory_durations [out] The duration between successive speed elements in the
 * profile. In seconds.
 */
void app_trajectory_planner_generatePositionTrajectoryTimeProfile(float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
                                                                  unsigned int num_elements,
                                                                  float trajectory_durations[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function balances the segment time durations of 2 separate speed profiles. The segment
 * durations of profile1 and profile2 will be made equal - to the LONGEST duration at each
 * point along the profiles. This is done by modifying the final speed elements.
 *
 * Note: Since displacement is the distance between speed nodes, there is 1 fewer element
 * than the speed[] arrays. This is normal.
 *
 * @pre The arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS.
 *
 * @param durations1 [in] The time durations corresponding to the length of time between
 * successive speed points in trajectory 1. In seconds.
 *
 * @param durations2 [in] The time durations corresponding to the length of time between
 * successive speed points in trajectory 2. In seconds.
 *
 * @param displacement1 [in] The displacement corresponding to the distance between
 * successive speed elements in trajectory 1. In meters.
 *
 * @param displacement2 [in] The displacement corresponding to the distance between
 * successive speed elements in trajectory 2. In meters.
 *
 * @param num_elements [in] The number of elements in both trajectory 1 and 2.
 *
 * @param speeds1 [in/out] The speed profile corresponding to trajectory 1. Speed must have a
 * denominator value of seconds.
 *
 * @param speeds2 [in/out] The speed profile corresponding to trajectory 2. Speed must have a
 * denominator value of seconds.
 *
 * @param complete_time_profile [out] The complete time profile of the two balanced
 * trajectories. This profile is absolute time to each point on the profile starting from
 * zero. In seconds.
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_modifySpeedsToMatchDuration(float displacement1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], float displacement2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
                                                   float durations1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], float durations2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], float num_elements,
                                                   float speeds1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], float speeds2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
                                                   float complete_time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

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
 * @param variable_period_trajectory [in] The variable-period trajectory that will be used
 * to generate the constant-period trajectory.
 *
 * @param interpolation_period [in] The interpolation period of the constant-period
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
app_trajectory_planner_interpolateConstantPeriodPositionTrajectory(PositionTrajectory_t *variable_period_trajectory,
                                                                   float interpolation_period,
                                                                   unsigned int *num_elements,
                                                                   PositionTrajectory_t *constant_period_trajectory);

/**
 * Function generates a constant interpolation period trajectory from the path parameters
 * specified in the constant parameterization trajectory. The function creates a constant
 * parameterization trajectory then uses linear interpolation to calculate the equivalent
 * constant interpolation period trajectory
 *
 * NOTE: Constant interpolation period means that the duration of time between each point
 * on the trajectory is CONSTANT. This is different from a constant parameterization
 * trajectory which has segments defined at constant polynomial evaluation intervals.
 * Constant arc-length trajectories (not used here) are trajectories where the distance
 * travelled in each segment in constant.
 *
 * @param interpolation_period [in] The duration of time between successive trajectory
 * elements. In seconds.
 *
 * @param path_parameters [in/out] The path parameters that define the trajectory. The
 * input path_parameters.num_elements defines the number of constant-parameterization
 * elements, and the returned path_parameters.num_elements defines the number of
 * constant-period elements in the trajectory,
 *
 * @param constant_period_trajectory [out] A constant interpolation period trajectory
 * generated by the specified path parameters.
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_generateConstantInterpolationPeriodPositionTrajectory(float interpolation_period,
                                                                             FirmwareRobotPathParameters_t *path_parameters,
                                                                             PositionTrajectory_t *constant_period_trajectory);

/**
 * Function generates a velocity trajectory composed of X/Y/angular velocities and their
 * time dependence.
 *
 * @param position_trajectory [in] A completely defined position trajectory.
 *
 * @param num_elements [in] The number of elements in the trajectory.
 *
 * @param velocity_trajectory [out] A completely defined velocity trajectory that has
 * equally time-spaced elements. The velocity trajectory has the name number of elements
 * as the input position trajectory.
 */
void
app_trajectory_planner_generateVelocityTrajectory(PositionTrajectory_t *position_trajectory, unsigned int num_elements,
                                                  VelocityTrajectory_t *velocity_trajectory);

/**
 * Function modifies the value of the argument final speed so that the time duration
 * between the initial and final speeds is equal to the parameter duration. This is done
 * assuming constant acceleration over the given displacement.
 *
 * @param initial_speed [in] The initial speed at the start of a segment. In m/s.
 *
 * @param duration [in] The duration of time between the initial and final speed points.
 * This is also the time required to traverse the displacement parameter. In seconds.
 *
 * @param displacement [in] The distance between the initial and final velocity points. In
 * meters.
 *
 * @param final_speed [in/out] A pointer to the final speed at the end of the segment. In
 * m/s.
 */
void app_trajectory_planner_modifySpeedToMatchDuration(float initial_speed, float duration, float displacement,
                                                       float *final_speed);
