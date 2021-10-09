#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/chip_intent.h"

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
    explicit ChipTactic(bool loop_forever);

    ChipTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the params for this tactic that cannot be derived from the world
     *
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    void updateControlParams(const Point& chip_origin, const Angle& chip_direction,
                             double chip_distance_meters);

    /**
     * Updates the control parameters for this ChipTactic.
     *
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     */
    void updateControlParams(const Point& chip_origin, const Point& chip_target);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    void accept(TacticVisitor& visitor) const override;

    bool done() const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;
    void updateIntent(const TacticUpdate& tactic_update) override;

    FSM<ChipFSM> fsm;

    // Tactic parameters
    ChipFSM::ControlParams control_params;
};

COPY_CLASS(KickoffChipTactic, ChipTactic)