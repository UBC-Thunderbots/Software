#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"

class WingerSkill : public Skill
{
   public:
    WingerSkill(const TbotsProto::AiConfig& ai_config, std::shared_ptr<Strategy> strategy,
                double initial_score);

    double calculateViability(const Robot& robot, const World& world) override;

    bool done() const override;

    void updatePrimitive(const Robot& robot, const World& world,
                         const TacticUpdate& tactic_update) override;
};
