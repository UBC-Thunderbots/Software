#pragma once

#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * This tactic is for moving a robot closest to the goal line to the goal line. It will be
 * used by the penalty kick goalie to position itself at the start of the penalty kick.
 */
class MoveToGoalLineTactic : public Tactic
{
   public:
    /**
     * Creates a new MoveToGoalLineTactic
     */
    explicit MoveToGoalLineTactic();

    void updateWorldParams(const World &world) override;

    double calculateRobotCost(const Robot &robot, const World &world) const override;

    void accept(TacticVisitor &visitor) const override;
    bool done() const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;
    void updateIntent(const TacticUpdate &tactic_update) override;

    FSM<MoveToGoalLineFSM> fsm;
};
