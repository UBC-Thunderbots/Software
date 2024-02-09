#include "software/ai/hl/stp/skill/shoot/shoot_skill.h"

#include "software/util/generic_factory/generic_factory.h"

double ShootSkill::getViability(const Robot& robot, const World& world) const
{
    std::optional<Shot> best_shot = (*strategy_)->getBestShot(robot);
    if (!best_shot)
    {
        return 0;
    }
    return 1;
}

void ShootSkill::updatePrimitive(const Robot& robot, const World& world,
                                 const SetPrimitiveCallback& set_primitive)
{
    if (!fsm_map_.contains(robot.id()))
    {
        reset(robot);
    }

    fsm_map_[robot.id()]->process_event(
        ShootSkillFSM::Update(ShootSkillFSM::ControlParams{},
                              SkillUpdate(robot, world, strategy_, set_primitive)));
}

void ShootSkill::reset(const Robot& robot)
{
    fsm_map_[robot.id()] = std::make_unique<FSM<ShootSkillFSM>>(DribbleSkillFSM());
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, ShootSkill, std::shared_ptr<Strategy>> factory;
