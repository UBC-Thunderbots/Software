#include "software/ai/hl/stp/skill/keep_away/keep_away_skill.h"

#include "software/util/generic_factory/generic_factory.h"

double KeepAwaySkill::getViability(const Robot& robot, const World& world) const
{
    return 1;
}

void KeepAwaySkill::updatePrimitive(const Robot& robot, const World& world,
                                    const SetPrimitiveCallback& set_primitive)
{
    if (!fsm_map_.contains(robot.id()))
    {
        reset(robot);
    }

    fsm_map_[robot.id()]->process_event(
        KeepAwaySkillFSM::Update(KeepAwaySkillFSM::ControlParams{},
                                 SkillUpdate(robot, world, strategy_, set_primitive)));
}

void KeepAwaySkill::reset(const Robot& robot)
{
    fsm_map_[robot.id()] = std::make_unique<FSM<KeepAwaySkillFSM>>(DribbleSkillFSM());
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, KeepAwaySkill, std::shared_ptr<Strategy>>
    factory;
