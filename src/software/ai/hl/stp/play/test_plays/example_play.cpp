#include "software/ai/hl/stp/play/test_plays/example_play.h"

#include "software/ai/hl/stp/skill/move/move_skill.h"
#include "software/ai/hl/stp/tactic/assigned_skill/assigned_skill_tactic.hpp"
#include "software/util/generic_factory/generic_factory.h"

ExamplePlay::ExamplePlay(std::shared_ptr<Strategy> strategy) : Play(false, strategy) {}

void ExamplePlay::getNextTactics(TacticCoroutine::push_type &yield,
                                 const WorldPtr &world_ptr)
{
    std::vector<std::shared_ptr<AssignedSkillTactic<MoveSkill>>> move_skill_tactics(
        DIV_A_NUM_ROBOTS);
    std::generate(move_skill_tactics.begin(), move_skill_tactics.end(), [&]() {
        return std::make_shared<AssignedSkillTactic<MoveSkill>>(strategy);
    });

    // Continue to loop to demonstrate the example play indefinitely
    do
    {
        // The angle between each robot spaced out in a circle around the ball
        Angle angle_between_robots =
            Angle::full() / static_cast<double>(move_skill_tactics.size());

        for (size_t k = 0; k < move_skill_tactics.size(); k++)
        {
            move_skill_tactics[k]->updateControlParams(
                {.destination = world_ptr->ball().position() +
                                Vector::createFromAngle(angle_between_robots *
                                                        static_cast<double>(k + 1)),
                 .final_orientation =
                     (angle_between_robots * static_cast<double>(k + 1)) + Angle::half(),
                 .final_speed = 0});
        }

        // yield the Tactics this Play wants to run, in order of priority
        // If there are fewer robots in play, robots at the end of the list will not be
        // assigned
        TacticVector result = {};
        result.insert(result.end(), move_skill_tactics.begin(), move_skill_tactics.end());
        yield({result});

    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ExamplePlay, std::shared_ptr<Strategy>> factory;
