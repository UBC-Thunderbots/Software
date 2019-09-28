#pragma once

#include "software/ai/intent/intent.h"
#include "software/ai/primitive/move_primitive.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"

class MoveIntent : public Intent, public MovePrimitive
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Move Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     * @param enable_dribbler Whether or not to enable the dribbler
     * @param slow Whether or not to move slower (1 m/s)
     * @param autokick This will enable the "break-beam" on the robot, that will
     *                        trigger the kicker to fire as soon as the ball is in front
     *                        of it
     */
    explicit MoveIntent(unsigned int robot_id, const Point& dest,
                        const Angle& final_angle, double final_speed,
                        unsigned int priority, bool enable_dribbler = false,
                        bool slow = false, AutokickType autokick = NONE);

    std::string getIntentName(void) const override;

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares MoveIntents for equality. MoveIntents are considered equal if all
     * their member variables are equal.
     *
     * @param other the MoveIntents to compare with for equality
     * @return true if the MoveIntents are equal and false otherwise
     */
    bool operator==(const MoveIntent& other) const;

    /**
     * Compares MoveIntents for inequality.
     *
     * @param other the MoveIntent to compare with for inequality
     * @return true if the MoveIntents are not equal and false otherwise
     */
    bool operator!=(const MoveIntent& other) const;
};
