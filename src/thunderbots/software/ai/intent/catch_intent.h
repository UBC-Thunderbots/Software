#pragma once

#include "ai/intent/intent.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "ai/primitive/catch_primitive.h"

class CatchIntent : public Intent, public CatchPrimitive
{
   public:
    /**
     * Creates a new Catch Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param priority The priority that the intent has relative to other intents (to be defined)
     */
    // TODO: Add parameter override field/object/struct
    explicit CatchIntent(unsigned int robot_id, const Point &dest,
                        const Angle &final_angle, double final_speed, int priority);

    /**
     * Creates a new Catch Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     */
    // TODO: Add parameter override field/object/struct
    explicit CatchIntent(unsigned int robot_id, const Point &dest,
                        const Angle &final_angle, double final_speed);

    std::string getIntentName() const override;

    int getPriority() const;

   private:
    int priority;
};
