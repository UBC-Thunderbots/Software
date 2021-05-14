#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_fsm.h"

/**
 * TODO: documentation
 */
class MoveToGoalLineTactic : public Tactic
{
public:
    /**
     * Creates a new MoveToGoalLineTactic
     *
     * @param goalie_tactic_config The config to fetch parameters from
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
