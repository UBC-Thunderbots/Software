#pragma once

#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/move_goalie_to_goal_line/move_goalie_to_goal_line_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * This tactic is for moving the goalie to the goal line. It will be used by the
 * penalty kick goalie to position itself at the start of the penalty kick.
 *
 * This tactic moves the goalie to the goal line in any game state.
 */
class MoveGoalieToGoalLineTactic : public Tactic
{
   public:
    /**
     * Creates a new MoveGoalieToGoalLineTactic
     */
    explicit MoveGoalieToGoalLineTactic();

    void updateWorldParams(const World &world) override;

    double calculateRobotCost(const Robot &robot, const World &world) const override;

    void accept(TacticVisitor &visitor) const override;
    bool done() const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;
    void updateIntent(const TacticUpdate &tactic_update) override;

    FSM<MoveGoalieToGoalLineFSM> fsm;
};
