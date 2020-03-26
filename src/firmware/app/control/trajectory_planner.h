#pragma once

#include "firmware/shared/math/polynomial_2d.h"
#include "firmware/shared/math/vector_2d.h"

#define __TRAJECTORY_PLANNER_MAX_NUM_SEGMENTS__ 6000 // The maximum size of the array containing trajectory elements
#define __TRAJECTORY_PLANNER_MAX_PATH_LENGTH__ 18 // [meters]

// Struct that defines a single point on a trajectory
// Includes the Position,and Timedata corresponding to that point 
typedef struct TrajectoryElement{
    Vector2d_t position;
    double time;
} TrajectoryElement_t;

typedef struct Trajectory{
    TrajectoryElement_t* trajectory_elements;
    unsigned int num_elements;
} Trajectory_t;

/**
 * Returns a planned trajectory as a list of TrajectoryElement's.
 * 
 * Key assumptions of this planner are:
 *  - Trajectories are time-optimal assuming INFINITE JERK capability of the robot
 *  - No speed on the trajectory is larger than the 'max_allowable_speed' parameter.
 *  - No acceleration value between points on the trajectory can be larger than the 'max_allowable_acceleration' input parameter
 *  - Assuming the grip-limit of the robot IS THE SAME AS THE MAX ACCELERATION and at no point on the path can the sum of centripital and acceleration force be greather than 'max_allowable_acceleration'
 *  
 *  Trajectory generation is done by assuming constant acceleration capability.
 *      - The generator will assume maximum acceleration for the robot between each segment on the path. If using max acceleration breaches either the speed limit of the robot, or the centripital acceleration limit defind by the curvature and robot speed - the planner will assume the max acceleration that will remain bellow this limit.
 *      - To ensure the robot is capable of deccelerating to reach ant sudden changes in the path, the trajectory is checked for backwards continuity
 *
 * @pre t_start < t_end
 * @pre num_segments > 2
 * @pre max_allowable_acceleration > 0
 * @pre max_allowable_speed > 0
 * @pre init_speed > 0
 * @pre final_speed > 0
 * 
 *
 * @param path [in] The 3D polynomial representing the geometric path
 * @param t_start [in] The path parameter value indicating the beggining of the considered path
 * @param t_end [in] The path parameter value indicating the end of the considered path
 * @param num_segments [in] The number of segments to discreteize the trajectory into. Must be greater than 2 segments.
 *                      * THE NUMBER OF SEGMENTS MUST BE UNDER __TRAJECTORY_PLANNER_MAX_NUMER_SEGMENTS__
 * @param max_allowable_acceleration [in] The maximum acceleration allowed at any point along the trajectory. This factor limits the maximum
 *                                         delta-velocity and also the max speed around curves due to centripital acceleration [m/s^2]
 * @param max_allowable_speed [in] The maximum speed allowable at any point along the path [m/s]
 * @param initial_speed [in] The initial speed at the start of the trajectory [m/s]
 * @param final_speed [in] The final speed at the end of the trajectory [m/s]
 */
TrajectoryElement_t* generate_constant_arc_length_segmentation(Polynomial2dOrder3_t path, double t_start, double t_end, 
                                                    unsigned int num_segments, double max_allowable_acceleration, double max_allowable_speed, double initial_speed, 
                                                    double final_speed);

/**
 * Returns a constant interpolation period (time) trajectory based on an input constant arclength trajectory
 * 
 * @pre constant_arc_length_
 * 
 * @param constant_arclength_trajectory [in] The constant arclength trajectory used as reference for a constant interpolation period trajectory
 * @param num_segments [in] The number of segments in the constant_arclength_trajectory input parameter
 * @param interpolation_period [in] The constant change in time that corresponds to each trajectory segment
 * 
*/
Trajectory_t interpolate_constant_time_trajectory_segmentation(TrajectoryElement_t* constant_arclength_trajectory, unsigned int num_segments, const double interpolation_period);