#include "software/ai/hl/stp/tactic/tactic_fsm.h"

class Skill
{
public:
    virtual std::unique_ptr<TbotsProto::Primitive> get(const Robot& robot, const World& world,
            CreateMotionControl create_motion_control, std::shared_ptr<Strategy> strategy) = 0;

    void updateScore(double score);

private:
    virtual std::shared_ptr<Skill> getNextSkill(const World& world) final;
};
