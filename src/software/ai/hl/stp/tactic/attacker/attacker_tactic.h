#pragma once

#include "software/ai/hl/stp/tactic/attacker/attacker_skill_executor.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_state.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/rl/dqn.hpp"
#include "software/ai/rl/exploration_strategies/epsilon_greedy_strategy.hpp"
#include "software/ai/rl/replay/prioritized_replay_buffer.hpp"

/**
 * The Attacker is the main ball handler during offensive gameplay. It executes
 * sequences of skills (e.g. dribble, pass, kick, chip) to create chances and score goals.
 *
 * The Attacker selects which skills to execute according to a learned policy, which
 * gives the probability of taking a given action (skill) in a given state (the World).
 * We attempt to find an optimal policy using reinforcement learning algorithms.
 */
class AttackerTactic : public Tactic
{
   public:
    /**
     * Constructs an AttackerTactic.
     *
     * @param ai_config the AI configuration
     */
    explicit AttackerTactic(TbotsProto::AiConfig ai_config);

    AttackerTactic() = delete;

    /**
     * Returns the skill currently being executed by the AttackerTactic.
     *
     * @return the currently selected skill, or std::nullopt if no skill is selected
     */
    std::optional<AttackerSkill> getCurrentSkill() const;

    /**
     * Selects a skill to execute based on the current World state, following the
     * Attacker agent's current policy.
     *
     * @param world_ptr the current World
     *
     * @return the selected skill
     */
    AttackerSkill selectSkill(const WorldPtr& world_ptr);

    /**
     * Terminates the AttackerTactic, ending the current episode.
     *
     * @param world_ptr the final World of the episode
     */
    void terminate(const WorldPtr& world_ptr);

    /**
     * Updates the control parameters for this AttackerTactic.
     *
     * @param pass the pass to take, if the currently selected skill
     * is a passing skill
     */
    void updateControlParams(const Pass& pass);

    void accept(TacticVisitor& visitor) const override;

    bool done() const override;

    std::string getFSMState() const override;

   private:
    /**
     * Updates the current state with the given World and runs the learning
     * algorithm on the Attacker agent's DQN.
     *
     * @param new_world the current World
     * @param is_final whether this is the final World in the current episode
     */
    void updateDQN(const WorldPtr& new_world, bool is_final);

    /**
     * Computes the reward for the transition from the old World state to
     * the new World state.
     *
     * @param old_world the old World
     * @param new_world the new World
     *
     * @return the reward for this transition
     */
    float computeReward(const World& old_world, const World& new_world);

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    // DQN hyperparameter values
    static constexpr float DQN_LEARNING_RATE   = 0.001f;
    static constexpr float DQN_DISCOUNT_RATE   = 0.95f;
    static constexpr float DQN_SOFT_UPDATE_TAU = 0.005f;

    // Replay buffer hyperparameter values
    static constexpr unsigned int REPLAY_BUFFER_CAPACITY   = 16384;
    static constexpr unsigned int REPLAY_BUFFER_BATCH_SIZE = 128;
    static constexpr float REPLAY_BUFFER_MIN_PRIORITY      = 0.001f;
    static constexpr float REPLAY_BUFFER_ALPHA             = 0.6f;
    static constexpr float REPLAY_BUFFER_BETA              = 0.4f;

    // Epsilon-greedy hyperparameter values
    static constexpr double EPSILON_START      = 0.9;
    static constexpr double EPSILON_END        = 0.05;
    static constexpr double EPSILON_DECAY_RATE = 1000;

    DQN<AttackerState, AttackerSkill> dqn_;
    PrioritizedReplayBuffer<AttackerState, AttackerSkill> replay_buffer_;
    EpsilonGreedyStrategy<AttackerSkill> epsilon_greedy_strategy_;

    WorldPtr current_world_;
    std::optional<AttackerState> current_state_;
    std::optional<AttackerSkill> current_skill_;

    std::map<RobotId, AttackerSkillExecutor> skill_executors_;

    AttackerSkillExecutor::ControlParams control_params_;
    TbotsProto::AiConfig ai_config_;
};
