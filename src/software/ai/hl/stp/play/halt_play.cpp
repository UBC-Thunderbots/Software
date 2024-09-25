#include "software/ai/hl/stp/play/halt_play.h"

#include "software/ai/hl/stp/skill/stop/stop_skill.h"
#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactic.hpp"
#include "software/util/generic_factory/generic_factory.h"

HaltPlay::HaltPlay(std::shared_ptr<Strategy> strategy) : Play(false, strategy) {}

void HaltPlay::getNextTactics(TacticCoroutine::push_type &yield,
                              const WorldPtr &world_ptr)
{
    auto stop_skill_tactic_1 = std::make_shared<AssignedSkillTactic<StopSkill>>(strategy);
    auto stop_skill_tactic_2 = std::make_shared<AssignedSkillTactic<StopSkill>>(strategy);
    auto stop_skill_tactic_3 = std::make_shared<AssignedSkillTactic<StopSkill>>(strategy);
    auto stop_skill_tactic_4 = std::make_shared<AssignedSkillTactic<StopSkill>>(strategy);
    auto stop_skill_tactic_5 = std::make_shared<AssignedSkillTactic<StopSkill>>(strategy);
    auto stop_skill_tactic_6 = std::make_shared<AssignedSkillTactic<StopSkill>>(strategy);

    do
    {
        // yield the Tactics this Play wants to run, in order of priority
        yield({{stop_skill_tactic_1, stop_skill_tactic_2, stop_skill_tactic_3,
                stop_skill_tactic_4, stop_skill_tactic_5, stop_skill_tactic_6}});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltPlay, std::shared_ptr<Strategy>> factory;
