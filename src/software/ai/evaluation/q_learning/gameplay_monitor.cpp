#include "software/ai/evaluation/q_learning/gameplay_monitor.h"

#include "software/logger/logger.h"
#include "software/geom/algorithms/contains.h"

void GameplayMonitor::startStepObservation(WorldPtr world_ptr)
{
    step_start_world_ptr_ = std::move(world_ptr);
}

double GameplayMonitor::endStepObservation(WorldPtr world_ptr)
{
    CHECK(step_start_world_ptr_ != nullptr) 
        << "Tried to end step observation for GameplayMonitor, "
        << "but no step observation was started";

    double reward = 0;

    // Reward friendly team scoring
    if (contains(world_ptr->field().enemyGoal(), world_ptr->ball().position()))
    {
        reward += 1.0;
    }

    // Penalize enemy team scoring
    if (contains(world_ptr->field().friendlyGoal(), world_ptr->ball().position()))
    {
        reward -= 1.0;
    }

    // TODO: Reward keeping possession

    // TODO: Reward enemy team fouls, penalize friendly team fouls

    return reward;
}