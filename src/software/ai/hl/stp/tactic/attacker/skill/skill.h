#pragma once

#include <random>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"
#include "software/util/generic_factory/generic_factory.h"

#define DEFINE_SKILL_GET_FSM_STATE                                                       \
    std::string getCurrentState() const override                                         \
    {                                                                                    \
        return getCurrentFullStateName(fsm);                                             \
    }


/**
 * In the new framework
 */
class Skill
{
   public:
    Skill(const TbotsProto::AiConfig& ai_config, std::shared_ptr<Strategy> strategy,
          double default_score,
          std::optional<unsigned> randomization_seed = std::nullopt);

    virtual double calculateViability(const Robot& robot, const World& world);

    virtual void updatePrimitive(const TacticUpdate& tactic_update) = 0;

    virtual bool done() const = 0;

    virtual std::string getCurrentState() const;

    std::shared_ptr<Skill> getNextSkill(const Robot& robot, const World& world);

    virtual double getScore() const;
    void updateScore(double score);

   protected:
    TbotsProto::AiConfig ai_config_;
    std::shared_ptr<Strategy> strategy_;
    double score_;

   private:
    std::mt19937 random_generator;
    std::optional<std::vector<std::shared_ptr<Skill>>> children;

    static constexpr double DEFAULT_SKILL_SCORE   = 1.0;
    static constexpr unsigned MAX_VIABILITY_SCORE = 100;
};
