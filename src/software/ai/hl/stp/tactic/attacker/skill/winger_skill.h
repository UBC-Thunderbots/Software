#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"

class WingerSkill : public Skill
{
   public:
    WingerSkill(const TbotsProto::AiConfig& ai_config, std::shared_ptr<Strategy> strategy,
                double initial_score);

    double calculateViability(const Robot& robot, const World& world) override;

    std::string getCurrentState() const override
    {
        return std::string();
    }

    // TODO(#3075): Uncomment once the implementation of the WingerFSM is done
    // DEFINE_SKILL_GET_FSM_STATE

    bool done() const override;

    void updatePrimitive(const TacticUpdate& tactic_update) override;
};
