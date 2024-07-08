#pragma once

#include "software/ai/evaluation/q_learning/attacker_mdp_action.h"
#include "software/ai/evaluation/q_learning/attacker_mdp_feature_extractor.h"
#include "software/ai/evaluation/q_learning/attacker_mdp_reward_function.h"
#include "software/ai/evaluation/q_learning/bandits/epsilon_greedy_strategy.hpp"
#include "software/ai/evaluation/q_learning/bandits/softmax_strategy.hpp"
#include "software/ai/evaluation/q_learning/linear_q_function.hpp"
#include "software/ai/evaluation/q_learning/q_policy.hpp"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The Attacker is the main ball handler during offensive gameplay. It executes
 * sequences of Skills (e.g. dribble, pass, kick, chip) to create chances and score goals.
 *
 * The Attacker selects which Skills to execute according to a learned policy, which
 * gives the probability of taking a given action (Skill) in a given state (the World).
 * We attempt to find the optimal policy using reinforcement learning algorithms.
 */
class AttackerTactic : public Tactic
{
   public:
    /**
     * Creates a AttackerTactic.
     *
     * @param strategy the shared Strategy used by all of AI
     */
    explicit AttackerTactic(std::shared_ptr<Strategy> strategy);

    AttackerTactic() = delete;

    void accept(TacticVisitor& visitor) const override;

    std::string getFSMState() const override;

    bool done() const override;

    /**
     * Returns whether the AttackerTactic has temporarily suspended execution
     * of its current Skill.
     *
     * IMPORTANT: updatePrimitive should not be called on this tactic if it is
     * suspended. This is because a suspended tactic will not yield any primitives
     * to execute.
     *
     * @return true if the tactic has suspended execution, false otherwise
     */
    bool suspended() const;

    /**
     * If the AttackerTactic has temporarily suspended execution of its current Skill,
     * calling this method will update the AttackerTactic with the given world and try
     * to resume execution (i.e. leave the suspended state).
     *
     * If the AttackerTactic is not suspended, then calling this method effectively
     * does nothing.
     *
     * @param world_ptr the world pointer
     *
     * @return true if the tactic resumed execution and is not suspended,
     * false if the tactic failed to resume and is still suspended
     */
    bool tryResumingIfSuspended(const WorldPtr& world_ptr);

    /**
     * Terminate the current episode of the AttackerTactic and reset the tactic
     * for a new episode
     *
     * @param world_ptr the World at the end of the current episode
     */
    void terminate(const WorldPtr& world_ptr);

    /**
     * Gets a SkillState containing details about the current state of the skill
     * that the AttackerTactic is executing.
     *
     * @return the current state of the Skill that the tactic is executing
     */
    SkillState getSkillState() const;

    /**
     * Log visualize the SkillState for this AttackerTactic.
     *
     * @param world the current state of the world
     */
    void visualizeSkillState(const World& world);

   private:
    // The shared Strategy used by all of AI
    std::shared_ptr<Strategy> strategy_;

    // The Q-function of the attacker agent's policy
    std::shared_ptr<LinearQFunction<AttackerMdpState, AttackerMdpAction>> q_function_;

    // The action selection strategy of attacker agent's policy
    std::shared_ptr<SoftmaxStrategy<AttackerMdpState, AttackerMdpAction>>
        action_selection_strategy_;

    // The policy that the attacker agent will follow
    QPolicy<AttackerMdpState, AttackerMdpAction> policy_;

    // The reward function for the attacker agent
    AttackerMdpRewardFunction attacker_mdp_reward_function_;

    // The current skill that the attacker is executing
    std::unique_ptr<Skill> current_skill_;

    // File to load Q-function weights from for initialization
    inline static const std::string ATTACKER_MDP_Q_FUNCTION_INITIAL_WEIGHTS_FILE =
        "software/ai/evaluation/q_learning/attacker_mdp_q_function_weights.csv";

    // Name of file that Q-function weights will be saved to at runtime
    inline static const std::string ATTACKER_MDP_Q_FUNCTION_RUNTIME_WEIGHTS_FILE =
        "attacker_mdp_q_function_weights.csv";

    // Name that identifies the attacker's Q-function in Thunderscope
    inline static const std::string ATTACKER_MDP_Q_FUNCTION_NAME =
        "AttackerMdpLinearQFunction";

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    /**
     * Update the policy of the attacker agent.
     *
     * @param attacker_mdp_state the new state entered after the attacker agent
     *                           completed the last executed action
     */
    void updatePolicy(const AttackerMdpState& attacker_mdp_state);
};
