#pragma once
#include <vector>
#include "Vector2.h"

class Simulator;

/**
 * \class  Goal
 * \brief  A goal in the simulation.
 */
class Goal {
private:
    /**
     * \brief      Constructor.
     * \param[in]  position  The position of this goal.
     */
    explicit Goal(const Vector2 &position);
    explicit Goal(std::vector<Vector2> positions);
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
