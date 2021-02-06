#pragma once

#include "software/ai/intent/direct_primitive_intent.h"

class StopIntent : public DirectPrimitiveIntent
{
   public:
    /**
     * Creates a new Stop Intent
     *
     * Stops the robot with the option to coast to a stop rather than stop immediately
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param coast to coast to a stop or not
     */
    explicit StopIntent(unsigned int robot_id, bool coast);

    StopIntent() = delete;
};
