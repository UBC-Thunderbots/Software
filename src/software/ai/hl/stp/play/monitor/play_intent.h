#pragma once
#include "software/geom/point.h"
#include "software/world/world.h"
#include "software/logger/logger.h"

enum IntentType
{
    PASS = 0,
    SHOT = 1,
    DRIBBLE = 2
};

class PlayIntent
{
public:

    explicit PlayIntent(IntentType play_intent_type);

    IntentType getIntentType() const;

private:
    const IntentType intent;
};


