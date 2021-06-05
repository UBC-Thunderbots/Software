#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/segment.h"

class PivotKickTactic : public Tactic
{
   public:
    /**
     * Creates a new PivotKickTactic
     *
     * @param robot_navigation_obstacle_config The config
     */
    explicit PivotKickTactic();

    void updateWorldParams(const World& world) override;
    bool done() const override;

    /**
     * Update control params for this tactic
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param auto_chip_or_kick The command to autochip or autokick
     */
    void updateControlParams(const Point& kick_origin, const Angle& kick_direction,
                             AutoChipOrKick auto_chip_or_kick);

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

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;
    void updateIntent(const TacticUpdate& tactic_update) override;

    // Tactic parameters
    FSM<PivotKickFSM> fsm;
    PivotKickFSM::ControlParams control_params;
};
