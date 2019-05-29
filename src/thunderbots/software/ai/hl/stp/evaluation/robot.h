#ifndef AI_HL_STP_EVALUATION_ROBOT_H_
#define AI_HL_STP_EVALUATION_ROBOT_H_

#include <ai/world/ball.h>
#include <ai/world/robot.h>

#include "geom/point.h"

/**
 * This file contains independent Evaluation function to evaluate whether robot
 * orientation at position is within threshold
 */


namespace Evaluation
{
    /**
     * Determines whether or not the robot at position
     * is facing within threshold degrees of the specified target
     *
     * @param position current position of the robot
     * @param orientation current orientation of the robot
     * @param target destination coordinate of the robot
     * @param threshold the upper bound of angle for evaluation
     *
     * @return True if angle formed between orientation direction and the
     * direction from position to target is smaller than threshold angle,
     * false otherwise
     */
    bool robotOrientationWithinAngleThresholdOfTarget(Point position, Angle orientation,
                                                      Point target, Angle threshold);

    /**
     * Determines if a robot has possession of the ball. A robot is considered to have
     * possession if the ball is in a area close to the its dribbler.
     *
     * @param ball The ball the is wanted to be possessed
     * @param robot The Robot which wants to know if it has the ball.
     * @return True if the ball is close to the front dribbler and false otherwise
     */
    bool robotHasPossession(Ball ball, Robot robot);
}  // namespace Evaluation


#endif
