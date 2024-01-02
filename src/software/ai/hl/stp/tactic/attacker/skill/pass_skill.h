#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"

class PassSkill : public Skill
{
    public:
        PassSkill(const TbotsProto::AiConfig& ai_config, double initial_score);

        double calculateViability(const Robot& robot, const World& world, std::shared_ptr<Strategy> strategy) override;
        bool done() const override;
        void updatePrimitive(const Robot& robot, const World& world, const TacticUpdate& tactic_update,
                std::shared_ptr<Strategy> strategy) override;

    private:
        FSM<AttackerFSM> fsm;
        AttackerFSM::ControlParams control_params;
};
