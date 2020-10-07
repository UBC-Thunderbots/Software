#pragma once

#include "firmware/app/control/trajectory_planner.h"
#include "firmware/shared/math/polynomial_1d.h"
#include "firmware/shared/math/polynomial_2d.h"

/**
 * Function generates the segment lengths and the states at each node along the specified
 * constant-parameterization segments. This function works for both the linear and angular
 * parts of a trajectory because the input is a generic polynomial.
 *
 * @pre All arrays must be pre-allocated up to TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param t_start The starting parameterization value
 *
 * @param t_end The ending parameterization value
 *
 * @param path_1d The 1d polynomial representing the path along the trajectory
 *
 * @param num_elements The number of elements (nodes) to be in the trajectory
 *
 * @param node_values [out] The evaluation of the polynomial at each value of the constant
 * parameterization. This array is of length num_elements
 *
 * @param segment_lengths [out] The length of segments between each node specified by the
 * constant parameterization. This array is of length num_elements-1
 *
 * Node1  |--segment length---| Node2
 *   ->   *-------------------*  <-
 */
void app_trajectory_planner_impl_generate1dSegmentNodesAndLengths(
    float t_start, float t_end, Polynomial1dOrder3_t path_1d, unsigned int num_elements,
    float node_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);


/**
 * Function generates the length of segments in a 2d polynomial. It works as an wrapper
 * for the euclidean distance and values of 2d polynomials.
 *
 * @pre All arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param t_start The starting value of polynomial parameterization
 *
 * @param t_end The final value of polynomial parameterization
 *
 * @param path_2d The polynomial representing the trajectory in space
 *
 * @param num_elements The number of nodes(elements) of the generated trajectory
 *
 * @param x_values [out] The x value of each trajectory node. This array is of length
 * num_elements.
 *
 * @param y_values [out] The y value of each trajectory node. THis array is of length
 * num_elements
 *
 * @param segment_lengths [out] The length of each trajectory segment (euclidean of the XY
 * 2d polynomial). This array is of length num_elements-1
 */
void app_trajectory_planner_impl_generate2dSegmentNodesAndLengths(
    float t_start, float t_end, Polynomial2dOrder3_t path_2d, unsigned int num_elements,
    float x_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float y_values[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function that modifies an existing speed profile to be backwards continuous. This means
 * that it is possible to reach every speed node within the limits of deceleration
 *
 * @pre All arrays are pre-allocated up to at least TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param initial_speed [in] The initial speed at the beginning of the trajectory.  Units
 * of parameters must be in consistent magnitudes, Ex: distances in mm and acceleration in
 * mm/s.
 *
 * @param segment_lengths [in] The length of each segment in the trajectory.  Units of
 * parameters must be in consistent magnitudes, Ex: distances in mm and speed in mm/s.
 *
 * @param max_allowable_acceleration The maximum acceleration that can occur anywhere
 * on the trajectory
 *
 * @param num_segments The number of nodes(elements) in the trajectory
 *
 * @param speeds [in/out] The existing speed profile to be modified in-place to be
 * backwards continuous. This array is of length num_elements. The units of output are
 * consistent with the input parameters.
 *
 * @return A status indicating whether or not modification was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_impl_modifySpeedsToBeBackwardsContinuous(
    float initial_speed, float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_acceleration, unsigned int num_segments,
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function creates a forwards continuous speed profile based on the specified parameters.
 * The profile will never exceed the maximums speeds specified by the
 * max_allowable_speed_profile
 *
 * @pre All arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS
 *
 * @param final_speed  The final speed at the last element of the profile.  Units of
 * parameters must be in consistent magnitudes, Ex: distances in mm and speed in mm/s.
 *
 * @param segment_lengths [in] The length of each segment between data points on the speed
 * profile. In meters.
 *
 * @param max_allowable_speed_profile An array that limits the max speed at each
 * point on the profile. Units of parameters must be in consistent magnitudes, Ex:
 * distances in mm and speed in mm/s.
 *
 * @param max_allowable_acceleration The maximum allowable acceleration along any
 * segment in the profile. Units of parameters must be in consistent magnitudes, Ex:
 * distances in mm and acceleration in mm/s^2.
 *
 * @param initial_speed The initial speed at the first element of the profile.  Units
 * of parameters must be in consistent magnitudes, Ex: distances in mm and speed in mm/s.
 *
 * @param num_elements The number of elements in the speed profile
 *
 * @param speeds [out] The pre-allocated array that will be modified to contain a forwards
 * continuous speed profile that obeys the initial and final speeds and the
 * max_allowable_speed_profile that limits the speed at each point on the trajectory.
 * Units are consistent with the input parameters. This array if of length num_elements.
 *
 * @return A status indicating whether or not generation was successful
 */
TrajectoryPlannerGenerationStatus_t
app_trajectory_planner_impl_createForwardsContinuousSpeedProfile(
    float final_speed, float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_speed_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float max_allowable_acceleration, float initial_speed, unsigned int num_elements,
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function generates the absolute maximum speed that can occur at any points along the
 * trajectory. This maximum speed is determined by the limit of grip (in acceleration)
 * along any curved path, along with imposed speed limits of the user.
 *
 * @pre All arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS
 *
 * @param path The 2d polynomial that defines the path in 2d space.
 *
 * @param num_elements The number of elements that will make up the trajectory
 *
 * @param t_start The starting value of the parameterization
 *
 * @param t_end The ending value of the parameterization
 *
 * @param max_allowable_acceleration The maximum allowable acceleration along the
 * trajectory. Units of parameters must be in consistent magnitudes, Ex: distances in mm
 * and acceleration in mm/s^2.
 *
 * @param speed_cap The maximum allowable speed at any point on the trajectory. This
 * value is defined by the user and this speed limit will not be exceeded even if possible
 * given the physical possibility. Units of parameters must be in consistent magnitudes,
 * Ex: distances in mm and speed in mm/s.
 *
 * @param max_allowable_speed_profile [out] The array that will be modified in-place to
 * contain the absolute maximum allowable speed profile. The units here are consistent
 * with the input units. This array is of length num_elements.
 */
void app_trajectory_planner_impl_getMaximumSpeedProfile(
    Polynomial2dOrder3_t path, unsigned int num_elements, float t_start, float t_end,
    float max_allowable_acceleration, float speed_cap,
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
 * the distance travelled between successive speed elements.  Units of parameters must be
 * in consistent magnitudes, Ex: distances in mm and acceleration in mm/s^2.
 *
 * @param speeds [in] The speed profile defining the speed at each element.  Units of
 * parameters must be in consistent magnitudes, Ex: distances in mm and speed in mm/s.
 *
 * @param num_elements The number of elements in the speed profile.
 *
 * @param trajectory_durations [out] The duration between successive speed elements in the
 * profile. In seconds. This array is of length num_elements-1
 */
void app_trajectory_planner_impl_generatePositionTrajectoryTimeProfile(
    float segment_lengths[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float speeds[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], unsigned int num_elements,
    float trajectory_durations[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);

/**
 * Function balances the segment time durations of 2 separate speed profiles. The segment
 * durations of profile1 and profile2 will be made equal - to the LONGEST duration at each
 * point along the profiles. This is done by modifying the final speed elements.
 *
 * Note: Since displacement is the distance between speed nodes, there is 1 fewer element
 * than the speed[] arrays. This is normal.
 *
 * Note: The magnitude of units have to be consistent. Ie. Distances in mm and speeds in
 * mm/s. The output will be in the same units magnitude.
 *
 * @pre The arrays must be pre-allocated up to at least
 * TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS.
 *
 * @param displacement1 [in] The displacement corresponding to the distance between
 * successive speed elements in trajectory 1. In meters. This array is of length
 * num_elements-1.
 *
 * @param displacement2 [in] The displacement corresponding to the distance between
 * successive speed elements in trajectory 2. In meters. This array is of length
 * num_elements-1.
 *
 * @param durations1 [in] The time durations corresponding to the length of time between
 * successive speed points in trajectory 1. In seconds. This array is of length
 * num_elements-1.
 *
 * @param durations2 [in] The time durations corresponding to the length of time between
 * successive speed points in trajectory 2. In seconds. This array is of length
 * num_elements-1.
 *
 * @param num_elements The number of elements in both trajectory 1 and 2.
 *
 * @param speeds1 [in/out] The speed profile corresponding to trajectory 1. Speed must
 * have a denominator value of seconds. This array is of length num_elements.
 *
 * @param speeds2 [in/out] The speed profile corresponding to trajectory 2. Speed must
 * have a denominator value of seconds. This array is of length num_elements.
 *
 * @param complete_time_profile [out] The complete time profile of the two balanced
 * trajectories. This profile is absolute time to each point on the profile starting from
 * zero. In seconds. This array is of length num_elements.
 */
void app_trajectory_planner_impl_modifySpeedsToMatchLongestSegmentDuration(
    float displacement1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float displacement2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float durations1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float durations2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS], float num_elements,
    float speeds1[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float speeds2[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS],
    float complete_time_profile[TRAJECTORY_PLANNER_MAX_NUM_ELEMENTS]);


/**
 * Function modifies the value of the argument final speed so that the time duration
 * between the initial and final speeds is equal to the parameter duration. This is done
 * assuming constant acceleration over the given displacement.
 *
 * Note: The magnitude of units have to be consistent. Ex. Distances in mm and speeds in
 * mm/s. The output will be in the same units magnitude.
 *
 * @param initial_speed The initial speed at the start of a segment.
 *
 * @param duration The duration of time between the initial and final speed points.
 * This is also the time required to traverse the displacement parameter.
 *
 * @param displacement The distance between the initial and final velocity points.
 *
 * @return final_speed The final speed at the end of the segment.
 */
float app_trajectory_planner_impl_calculateSpeedToMatchDuration(float initial_speed,
                                                                float duration,
                                                                float displacement);
