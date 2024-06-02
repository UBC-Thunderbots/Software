#pragma once

#include "software/ai/evaluation/q_learning/attacker_mdp_action.h"
#include "software/ai/evaluation/q_learning/attacker_mdp_feature_extractor.h"
#include "software/ai/evaluation/q_learning/bandits/epsilon_greedy_strategy.hpp"
#include "software/ai/evaluation/q_learning/bandits/softmax_strategy.hpp"
#include "software/ai/evaluation/q_learning/linear_q_function.hpp"
#include "software/ai/evaluation/q_learning/gameplay_monitor.h"
#include "software/ai/evaluation/q_learning/q_policy.hpp"
#include "software/ai/hl/stp/tactic/tactic.h"

class AttackerTactic : public Tactic
{
   public:
    explicit AttackerTactic(std::shared_ptr<Strategy> strategy);

    AttackerTactic() = delete;

    void accept(TacticVisitor& visitor) const override;

    bool done() const override;

    std::string getFSMState() const override;

   private:
    std::shared_ptr<Strategy> strategy;

    std::shared_ptr<LinearQFunction<AttackerMdpState, AttackerMdpAction>> q_function_;
    
    std::shared_ptr<ActionSelectionStrategy<AttackerMdpState, AttackerMdpAction>>
        action_selection_strategy_;

    QPolicy<AttackerMdpState, AttackerMdpAction> policy_;

    GameplayMonitor gameplay_monitor_;

    std::unique_ptr<Skill> current_skill_;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;
};
