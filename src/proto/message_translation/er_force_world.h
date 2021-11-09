#pragma once

#include "software/world/ball.h"
#include "software/world/robot.h"
#include "extlibs/er_force_sim/src/amun/simulator/simball.h"
#include "extlibs/er_force_sim/src/amun/simulator/simrobot.h"

/**
 * convert a er force simulator ball to a ball object
 * 
 * @param sim_ball: the ball information retrieved from the er force simulator
 * 
 * @return the corresponding ball object to the er simulator ball 
 */

Ball createBall(world::SimBall sim_ball);

/**
 * Convert a er force simulator robot to a robot object
 * 
 * @param sim_robot: the robot information retrieved from the er force simulator 
 * 
 * @return the corresponding robot object to the er simulator robot 
 */
Robot createRobot(world::SimRobot sim_robot);