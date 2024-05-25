#include "software/ai/evaluation/q_learning/gameplay_monitor.h"

#include "software/logger/logger.h"

void GameplayMonitor::startStepObservation(WorldPtr world_ptr)
{
    step_start_world_ptr_ = std::move(world_ptr);
}

double GameplayMonitor::endStepObservation(WorldPtr world_ptr)
{
    CHECK(step_start_world_ptr_ != nullptr) 
        << "Tried to end step observation for GameplayMonitor, "
        << "but no step observation was started";

    // TODO: tune and improve reward function

    const double start_ball_pos = step_start_world_ptr_->ball().position().x();
    const double final_ball_pos = world_ptr->ball().position().x();
    const double ball_distance_travelled = final_ball_pos - start_ball_pos;

    return ball_distance_travelled / world_ptr->field().xLength();
}