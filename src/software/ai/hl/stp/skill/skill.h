#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/strategy.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"

#include <cstdlib>

class Skill
{
public:
    Skill(const TbotsProto::AiConfig& ai_config, double default_score,
            std::optional<unsigned> randomization_seed = std::nullopt);

    virtual void updatePrimitive(const Robot& robot, const World& world,
            const TacticUpdate& tactic_update, std::shared_ptr<Strategy> strategy) = 0;

    void updateScore(double score);

protected:
    TbotsProto::AiConfig ai_config;

private:
    virtual std::shared_ptr<Skill> getNextSkill(const World& world) final;

    double score;

    std::optional<std::vector<std::shared_ptr<Skill>>> children;

    static constexpr double DEFAULT_SKILL_SCORE = 1.0;
};
