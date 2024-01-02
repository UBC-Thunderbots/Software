#include "software/ai/hl/stp/tactic/attacker/skill/skill.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"

class DribbleSkill : public Skill
{
   public:
    DribbleSkill(const TbotsProto::AiConfig& ai_config, double initial_score);

    double calculateViability(const Robot& robot, const World& world, std::shared_ptr<Strategy> strategy) override;

    bool done() const override;

    void updatePrimitive(const Robot& robot, const World& world,
                         const TacticUpdate& tactic_update,
                         std::shared_ptr<Strategy> strategy) override;

   private:
    FSM<DribbleFSM> fsm;
    DribbleFSM::ControlParams control_params;
};
