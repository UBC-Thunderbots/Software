#pragma once

#include <random>

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/strategy.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"
#include "software/util/generic_factory/generic_factory.h"

class Skill
{
   public:
    Skill(const TbotsProto::AiConfig& ai_config, double default_score,
          std::optional<unsigned> randomization_seed = std::nullopt);

    virtual double calculateViability(const Robot& robot, const World& world) = 0;

    virtual void updatePrimitive(const Robot& robot, const World& world,
                                 const TacticUpdate& tactic_update,
                                 std::shared_ptr<Strategy> strategy) = 0;

    void updateScore(double score);

   protected:
    TbotsProto::AiConfig ai_config;

   private:
    virtual std::shared_ptr<Skill> getNextSkill(const Robot& robot,
                                                const World& world) final;

    double score;

    std::mt19937 random_generator;
    std::optional<std::vector<std::shared_ptr<Skill>>> children;

    static constexpr double DEFAULT_SKILL_SCORE   = 1.0;
    static constexpr unsigned MAX_VIABILITY_SCORE = 100;
};
