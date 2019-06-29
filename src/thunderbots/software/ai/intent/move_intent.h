#pragma once

#include "ai/flags.h"
#include "ai/intent/intent.h"
#include "ai/navigator/obstacle/obstacle.h"
#include "ai/primitive/move_primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

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
     * @param autokick This will enable the "break-beam" on the robot, that will
     *                        trigger the kicker to fire as soon as the ball is in front
     *                        of it
     */
    explicit MoveIntent(unsigned int robot_id, const Point& dest,
                        const Angle& final_angle, double final_speed,
                        unsigned int priority, bool enable_dribbler = false,
                        AutokickType autokick = NONE);

    std::string getIntentName(void) const override;

    /**
     * Sets MoveFlags of this intent.
     *
     * @param flags The MoveFlags to set to this intent.
     */
    void setMoveFlags(MoveFlags flags);

    /**
     * Returns the current MoveFlags of this intent.
     *
     * @return The current MoveFlags of this intent.
     */
    MoveFlags getMoveFlags();

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

    /*
     * Gets additional obstacles for this move intent
     *
     * @return additional obstacles for this move intent
     */
    std::vector<Obstacle> getAdditionalObstacles() const;

    /*
     * Adds an additional obstaclesfor this move intent
     *
     * @param an additional obstacle for this move intent
     */
    void getAdditionalObstacles(Obstacle o);

   private:
    /**
     * MoveFlags of this intent.
     */
    MoveFlags flags;
    // TODO: actually use move flags, this is a competition hack
    std::vector<Obstacle> additional_obstacles;
};
