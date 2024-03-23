#pragma once
#include "play_intent.h"
#include "shared/constants.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"
#include "software/world/world.h"

/**
 * Represents a class that monitors and computes a success score for a running play
 */
class PlayMonitor
{
   public:
    explicit PlayMonitor();

    void startMonitoring(const World& initialWorld);
    double endMonitoring(const World& finalWorld);
    void updateWorld(const World& newWorld);

   private:
    double calculateIntentBallScore();
    double calculateIntentActionScore();

    double calculateCurrentPlayScore(const World& finalWorld);

    World world;
};
