#pragma once

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "software/world/ball.h"
#include "software/world/robot.h"

/**
 * convert a er force simulator ball to a ball object
 *
 * @param sim_ball: the ball information retrieved from the er force simulator
 * @param timestamp: the timestamp of the newly created ball
 *
 * @return the corresponding ball object to the er simulator ball
 */

Ball createBall(world::SimBall sim_ball, Timestamp timestamp);

/**
 * Convert a er force simulator robot to a robot object
 *
 * @param sim_robot: the robot information retrieved from the er force simulator
 * @param timestamp: the timestamp of the newly created ball
 *
 * @return the corresponding robot object to the er simulator robot
 */
Robot createRobot(world::SimRobot sim_robot, Timestamp timestamp);
