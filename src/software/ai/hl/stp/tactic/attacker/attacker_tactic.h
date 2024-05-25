#pragma once

#include "software/ai/evaluation/q_learning/attacker_mdp_action.h"
#include "software/ai/evaluation/q_learning/attacker_mdp_feature_extractor.h"
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

    /**
     * Evaluate the AttackerTactic and start a new iteration
     *
     * @param score score between [-1, 1] rating the success of the AttackerTactic
     * for the current iteration
     */
    void evaluate(double score);

   private:
    std::shared_ptr<Strategy> strategy;

    QPolicy<AttackerMdpState, AttackerMdpAction> attacker_mdp_policy_;

    std::unique_ptr<Skill> current_skill_;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;
};
