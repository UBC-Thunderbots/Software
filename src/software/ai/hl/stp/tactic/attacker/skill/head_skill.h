#pragma once

#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"

class HeadSkill : public Skill
{
   public:
    HeadSkill(const TbotsProto::AiConfig& ai_config, std::shared_ptr<Strategy> strategy,
              std::optional<unsigned> randomization_seed = std::nullopt);

    inline double calculateViability(const Robot& robot, const World& world) override
    {
        return 0.0;
    }

    inline bool done() const override
    {
        return true;
    }

    inline double getScore() const override
    {
        return 0.0;
    }

    void updatePrimitive(const TacticUpdate& tactic_update) override;

    std::string getCurrentState() const override;
};
