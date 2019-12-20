#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The ChipTactic will move the assigned robot to the given chip origin and then
 * chip the ball to the chip target.
 */

class ChipTactic : public Tactic
{
   public:
    /**
     * Creates a new ChipTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit ChipTactic(const Ball& ball, bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the world parameters for this ChipTactic.
     *
     * @param ball The ball being kicked
     */
    void updateWorldParams(const Ball& ball);

    /**
     * Updates the control parameters for this ChipTactic.
     *
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    void updateControlParams(Point chip_origin, Point chip_target,
                             double chip_distance_meters);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

    void accept(TacticVisitor& visitor) const override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Tactic parameters
    Ball ball;
    Point chip_origin;
    Point chip_target;
    double chip_distance_meters;
};
