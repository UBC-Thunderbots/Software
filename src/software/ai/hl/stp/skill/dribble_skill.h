#include "software/ai/hl/stp/skill/skill.h"

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"

class DribbleSkill : public Skill
{
public:
    DribbleSkill(const TbotsProto::AiConfig& ai_config, double initial_score);

    void updatePrimitive(const Robot& robot, const World& world,
            const TacticUpdate& tactic_update, std::shared_ptr<Strategy> strategy) override;
private:
    FSM<DribbleFSM> fsm;
    DribbleFSM::ControlParams control_params;
};
