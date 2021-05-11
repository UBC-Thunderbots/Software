#pragma once

#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

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
    explicit MoveToGoalLineTactic(std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config);

    MoveToGoalLineTactic() = delete;

    void updateWorldParams(const World &world) override;

    double calculateRobotCost(const Robot &robot, const World &world) const override;

    void accept(TacticVisitor &visitor) const override;
    bool done() const override;
    bool isGoalieTactic() const override;

private:
    void calculateNextAction(ActionCoroutine::push_type &yield) override;
    void updateIntent(const TacticUpdate &tactic_update) override;

    FSM<GoalieFSM> fsm;
    std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config;
    GoalieFSM::ControlParams control_params;
};
