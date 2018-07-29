#pragma once

/**
 * An intent is a simple "thing" a robot or player may want to do. It specifies WHAT a
 * robot should do, not
 * necessarily exactly how it will do it. Examples are shooting at a point (for example
 * the enemy net), or moving
 * to a location without colliding with anything.
 *
 * Intents can be considered to be one level above Primitives in terms of abstraction.
 * Primitives are simply the
 * smallest/simplest action a robot can take and are not concerned with gameplay logic,
 * while Intents
 * do deal with gameplay logic.
 */
class Intent
{
   public:
    /**
     * Create a new Intent.
     */
    explicit Intent();
};
