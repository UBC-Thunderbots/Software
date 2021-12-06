#include "Goal.h"

#include <utility>
#include <stdexcept>

Goal::Goal(const Vector2 &position) : position_(position)
{
    positions_.push_back(position);
    speedAtPosition_.push_back(0.f);
}

Goal::Goal(std::vector<Vector2> positions) : positions_(std::move(positions))
{
    speedAtPosition_ = std::vector<float>(positions_.size(), 0.f);
}

Goal::Goal(std::vector<Vector2> positions, std::vector<float> speedAtPostition) :
        positions_(std::move(positions)),
        speedAtPosition_(std::move(speedAtPostition))
{
    if (positions_.size() != speedAtPosition_.size())
    {
        throw std::invalid_argument("positions and speedAtPostition arrays have to be the same size");
    }
}

Vector2 Goal::getNextGoalPostion() {
    currGoalIndex++;
    return getCurrentGoalPosition();
}

Vector2 Goal::getCurrentGoalPosition() {
    if (currGoalIndex >= positions_.size())
    {
        return Vector2();
    }
    else
    {
        return positions_[currGoalIndex];
    }
}

bool Goal::isGoingToFinalGoal() {
    if(currGoalIndex >= positions_.size() - 1)
    {
        return true;
    }
    return false;
}

float Goal::getDesiredSpeedAtCurrentGoal() {
    if (currGoalIndex >= speedAtPosition_.size())
    {
        return 0.f;
    }
    else
    {
        return speedAtPosition_[currGoalIndex];
    }
}
