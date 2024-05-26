#pragma once

#include "software/ai/evaluation/q_learning/attacker_mdp_action.h"
#include "software/ai/evaluation/q_learning/attacker_mdp_feature_extractor.h"
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

    QPolicy<AttackerMdpState, AttackerMdpAction> attacker_mdp_policy_;

    GameplayMonitor gameplay_monitor_;

    std::unique_ptr<Skill> current_skill_;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;
};
