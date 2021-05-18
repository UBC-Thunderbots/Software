#include "software/ai/hl/stp/play/example_play.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

ExamplePlay::ExamplePlay(std::shared_ptr<const PlayConfig> config) : Play(config, false)
{
}

bool ExamplePlay::isApplicable(const World &world) const
{
    // This play is never applicable so it will never be chosen during gameplay
    // This play can be run for testing by using the Play override
    return false;
}

bool ExamplePlay::invariantHolds(const World &world) const
{
    return true;
}

void ExamplePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    // Create MoveTactics that will loop forever
    std::vector<std::shared_ptr<MoveTactic>> move_tactics(DIV_A_NUM_ROBOTS);
    std::generate(move_tactics.begin(), move_tactics.end(),
                  []() { return std::make_shared<MoveTactic>(true); });

    // Continue to loop to demonstrate the example play indefinitely
    do
    {
        // The angle between each robot spaced out in a circle around the ball
        Angle angle_between_robots =
            Angle::full() / static_cast<double>(world.friendlyTeam().numRobots());

        for (size_t k = 0; k < move_tactics.size(); k++)
        {
            move_tactics[k]->updateControlParams(
                world.ball().position() +
                    Vector::createFromAngle(angle_between_robots *
                                            static_cast<double>(k + 1)),
                (angle_between_robots * static_cast<double>(k + 1)) + Angle::half(), 0);
        }

        // yield the Tactics this Play wants to run, in order of priority
        // If there are fewer robots in play, robots at the end of the list will not be
        // assigned
        TacticVector result = {};
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});

    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ExamplePlay, PlayConfig> factory;
