#pragma once

#include <string>

const static std::string MOVE_INTENT_NAME = "Move Intent";

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
    virtual std::string getIntentName() const = 0;

    /**
     * Returns the id of the robot this Intent corresponds to
     * @return the id of the robot this Intent corresponds to
     */
    virtual unsigned int getRobotId() const = 0;

    virtual ~Intent() = default;
};
