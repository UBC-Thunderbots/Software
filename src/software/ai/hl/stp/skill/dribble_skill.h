#include "software/ai/hl/stp/skill/skill.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"

class DribbleSkill : public Skill
{
public:
    std::unique_ptr<TbotsProto::Primitive> get(const Robot& robot, const World& world,
            CreateMotionControl create_motion_control, std::shared_ptr<Strategy> strategy) override;
private:
    FSM<DribbleFSM> fsm;
    DribbleFSM::ControlParams control_params;
};
