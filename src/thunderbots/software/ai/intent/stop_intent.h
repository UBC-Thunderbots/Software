#pragma once

#include "ai/intent/intent.h"
#include "ai/primitive/stop_primitive.h"

class StopIntent : public Intent, public StopPrimitive
{
public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Stop Intent
     *
     * Stops the robot with the option to coast to a stop rather than stop immediately
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param coast to coast to a stop or not
     */
    explicit StopIntent(unsigned int robot_id, bool coast);

    std::string getIntentName(void) const override;
};

