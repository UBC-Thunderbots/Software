#pragma once

#include "software/ai/intent/intent.h"
#include "software/primitive/stop_primitive.h"

class StopIntent : public StopPrimitive, public Intent
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
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit StopIntent(unsigned int robot_id, bool coast, unsigned int priority);

    std::string getIntentName(void) const override;

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares StopIntents for equality. StopIntents are considered equal if all
     * their member variables are equal.
     *
     * @param other the StopIntents to compare with for equality
     * @return true if the StopIntents are equal and false otherwise
     */
    bool operator==(const StopIntent& other) const;

    /**
     * Compares StopIntents for inequality.
     *
     * @param other the StopIntent to compare with for inequality
     * @return true if the StopIntents are not equal and false otherwise
     */
    bool operator!=(const StopIntent& other) const;
};
