#pragma once

#include "software/world/world.h"

class GameplayMonitor
{
   public:
    void startStepObservation(WorldPtr world_ptr);

    double endStepObservation(WorldPtr world_ptr);

   private:
    WorldPtr step_start_world_ptr_;
};