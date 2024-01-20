#pragma once
#include "play_intent.h"
#include "software/world/world.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "shared/constants.h"
#include "play_intent.h"
#include "software/logger/logger.h"

/**
 * Represents a class that monitors and computes a sucess score for a running play
 */
class PlayMonitor
{
public:
    explicit PlayMonitor(const PlayIntent& initialIntent);

    void startMonitoring(const World& initialWorld);
    double endMonitoring(const World &finalWorld);
    void updateWorld(const World& newWorld);
    void updatePlayIntent(PlayIntent newIntent);

private:
    double calculateIntentBallScore();
    double calculateIntentActionScore();

    double calculateCurrentPlayScore(const World & finalWorld);

    PlayIntent intent;
    World world;
};
