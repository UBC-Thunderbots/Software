#include "agent.h"

#include "extlibs/hrvo/simulator.h"

Agent::Agent(Simulator *simulator, const Vector2 &position, float radius,
             const Vector2 &velocity, const Vector2 &prefVelocity, float maxSpeed,
             float maxAccel, std::size_t goalNo, float goalRadius)
    : simulator_(simulator),
      position_(position),
      radius_(radius),
      velocity_(velocity),
      prefVelocity_(prefVelocity),
      maxSpeed_(maxSpeed),
      maxAccel_(maxAccel),
      goalNo_(goalNo),
      goalRadius_(goalRadius),
      reachedGoal_(false)
{
}

void Agent::update()
{
    const float dv = abs(newVelocity_ - velocity_);

    if (dv < maxAccel_ * simulator_->timeStep_)
    {
        velocity_ = newVelocity_;
    }
    else
    {
        velocity_ = (1.0f - (maxAccel_ * simulator_->timeStep_ / dv)) * velocity_ +
                    (maxAccel_ * simulator_->timeStep_ / dv) * newVelocity_;
    }

    position_ += velocity_ * simulator_->timeStep_;

    if (absSq(simulator_->goals_[goalNo_]->getCurrentGoalPosition() - position_) <
        goalRadius_ * goalRadius_)
    {
        // Is at current goal position
        if (simulator_->goals_[goalNo_]->isGoingToFinalGoal())
        {
            reachedGoal_ = true;
        }
        else
        {
            simulator_->goals_[goalNo_]->getNextGoalPostion();
            reachedGoal_              = false;
            simulator_->reachedGoals_ = false;
        }
    }
    else
    {
        reachedGoal_              = false;
        simulator_->reachedGoals_ = false;
    }
}

float Agent::getMaxAccel() const
{
    return maxAccel_;
}

const Vector2 &Agent::getVelocity() const
{
    return velocity_;
}

float Agent::getRadius() const
{
    return radius_;
}

const Vector2 &Agent::getPosition() const
{
    return position_;
}
