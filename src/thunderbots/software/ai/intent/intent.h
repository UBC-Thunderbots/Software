#pragma once

#include <string>

/**Used for initializing `INTENT_NAME`s and comparisons,
 * for example getIntent() == XX_INTENT_NAME
 */
const static std::string CATCH_INTENT_NAME           = "Catch Intent";
const static std::string CHIP_INTENT_NAME            = "Chip Intent";
const static std::string DIRECT_VELOCITY_INTENT_NAME = "Direct Velocity Intent";
const static std::string KICK_INTENT_NAME            = "Kick Intent";
const static std::string MOVE_INTENT_NAME            = "Move Intent";
const static std::string PIVOT_INTENT_NAME           = "Pivot Intent";

/**
 * An intent is a simple "thing" a robot or player may want to do. It specifies WHAT a
 * robot should do, not necessarily exactly how it will do it. Examples are shooting at
 * a point (for example the enemy net), or moving to a location.
 *
 * Intents can be considered to be one level above Primitives in terms of abstraction.
 * Primitives are simply the smallest/simplest action a robot can take and are not
 * concerned with gameplay logic, while Intents do deal with gameplay logic.
 *
 * We define an Abstract base class for Intents, despite not providing very many
 * pure-virtual functions, so that we can create generic structures of Intents.
 * For example we can create vectors of generic Intent objects (using pointers)
 * which is easier than a separate container for each type of Intent.
 */
class Intent
{
   public:
    /**
     * Returns the name of this Intent
     *
     * @return the name of this Intent
     */
    virtual std::string getIntentName(void) const = 0;

    /**
     * Returns the priority of this Intent
     * @return the priority of this Intent
     */
    int getPriority(void) const;

    /**
     * Sets the priority of this Intent
     */
    void setPriority(int);

    virtual ~Intent() = default;

   private:
    /**
     * priority of this intent
     * higher value => higher priority
     */
    int priority;
};
