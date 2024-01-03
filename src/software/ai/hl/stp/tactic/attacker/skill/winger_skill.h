#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"

class WingerSkill : public Skill
{
   public:
    WingerSkill(const TbotsProto::AiConfig& ai_config, std::shared_ptr<Strategy> strategy,
                double initial_score);

    double calculateViability(const Robot& robot, const World& world) override;

    // TODO(#3075): Update the return string output when the WingerSkill (& fsm) is
    // implemented
    std::string getFSMState() const override
    {
        return std::string();
    }

    bool done() const override;

    void updatePrimitive(const TacticUpdate& tactic_update) override;
};
