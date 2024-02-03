#include "software/ai/hl/stp/skill/shoot/shoot_skill.h"

#include "software/util/generic_factory/generic_factory.h"

double ShootSkill::getViability(const Robot& robot, const World& world) const
{
    if (!(*strategy_)->getBestShot(robot))
    {
        return 0;
    }
    return 1;
}

std::shared_ptr<Primitive> ShootSkill::getPrimitive(const Robot& robot,
                                                    const World& world)
{
    if (!fsm_map_.contains(robot.id()))
    {
        reset(robot);
    }

    std::shared_ptr<Primitive> new_primitive;
    fsm_map_[robot.id()]->process_event(ShootSkillFSM::Update(
        ShootSkillFSM::ControlParams{},
        SkillUpdate(robot, world, strategy_, [&](std::shared_ptr<Primitive> primitive) {
            new_primitive = std::move(primitive);
        })));
    return new_primitive;
}

void ShootSkill::reset(const Robot& robot)
{
    fsm_map_[robot.id()] = std::make_unique<FSM<ShootSkillFSM>>(DribbleSkillFSM());
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, ShootSkill, std::shared_ptr<Strategy>> factory;
