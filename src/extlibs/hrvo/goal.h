#pragma once
#include <vector>

#include "vector2.h"

class Simulator;

/**
 * A goal in the simulation.
 */
class Goal
{
   private:
    /**
     * @param position  The position of this goal.
     */
    explicit Goal(const Vector2 &position);

    /**
     * @param positions  The list of positions which the the robot should travel to in
     * order.
     */
    explicit Goal(std::vector<Vector2> positions);

    /**
     * @param positions  The list of positions which the the robot should travel to in
     * order.
     * @param speedAtPostition  The list of speed which the the robot should be at when it
     * reached the position. Must be in the same order of the positions
     */
    explicit Goal(std::vector<Vector2> positions, std::vector<float> speedAtPostition);

   public:
    Vector2 position_;
    // Goal positions in order Could be Queue
    std::vector<Vector2> positions_;
    std::vector<float> speedAtPosition_;
    unsigned int currGoalIndex = 0;

    Vector2 getNextGoalPostion();
    Vector2 getCurrentGoalPosition();
    float getDesiredSpeedAtCurrentGoal();
    bool isGoingToFinalGoal();


    friend class Agent;
    friend class Simulator;
};
