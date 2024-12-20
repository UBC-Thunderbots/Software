#pragma once

#include "software/ai/rl/dqn.hpp"
#include "software/ai/rl/exploration_strategies/epsilon_greedy_strategy.hpp"
#include "software/ai/hl/stp/tactic/attacker/attacker_state.h"

#include "software/ai/hl/stp/tactic/attacker/attacker_skill_executor.h"
#include "software/ai/hl/stp/tactic/tactic.h"

class AttackerTactic : public Tactic
{
   public:
    explicit AttackerTactic(TbotsProto::AiConfig ai_config);

    AttackerTactic() = delete;

    std::optional<AttackerSkill> getCurrentSkill() const;

    AttackerSkill selectSkill(const WorldPtr& world_ptr);

    void terminate(const WorldPtr& world_ptr);

    void updateControlParams(const Pass& pass);

    void accept(TacticVisitor& visitor) const override;

    bool done() const override;

    std::string getFSMState() const override;

   private:
    /**
     *
     * @param new_world the current World
     * @param is_final whether this is the final World in the current episode
     */
    void updateDQN(const WorldPtr& new_world, bool is_final);

    float computeReward(const World& old_world, const World& new_world);

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    static constexpr float DQN_LEARNING_RATE   = 0.001f;
    static constexpr float DQN_DISCOUNT_RATE   = 0.95f;
    static constexpr float DQN_SOFT_UPDATE_TAU = 0.005f;

    static constexpr unsigned int REPLAY_BUFFER_CAPACITY = 10000;
    static constexpr unsigned int TRANSITION_BATCH_SIZE  = 128;

    static constexpr float EXPLORATION_EPSILON = 0.1f;

    DQN<AttackerState, AttackerSkill> dqn_;
    ReplayBuffer<AttackerState, AttackerSkill> replay_buffer_;
    EpsilonGreedyStrategy<AttackerSkill> epsilon_greedy_strategy_;

    WorldPtr current_world_;
    std::optional<AttackerState> current_state_;
    std::optional<AttackerSkill> current_skill_;

    std::map<RobotId, AttackerSkillExecutor> skill_executors_;

    AttackerSkillExecutor::ControlParams control_params_;
    TbotsProto::AiConfig ai_config_;
};
