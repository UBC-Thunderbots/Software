#include "software/ai/hl/stp/play/example_play.h"

#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

ExamplePlay::ExamplePlay(std::shared_ptr<const PlayConfig> config) : Play(config) {}

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
    TacticVector tactic_vector = {{
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true),
    }};

    // Continue to loop to demonstrate the example play indefinitely
    do
    {
        // The angle between each robot spaced out in a circle around the ball
        Angle angle_between_robots =
            Angle::full() / static_cast<double>(world.friendlyTeam().numRobots());

        for (int k = 0; k < tactic_vector[0].length(); k++)
        {
            tactic_vector[0]->updateControlParams(
                world.ball().position() +
                    Vector::createFromAngle(angle_between_robots * (k + 1)),
                (angle_between_robots * (k + 1)) + Angle::half(), 0);
        }

        // yield the Tactics this Play wants to run, in order of priority
        // If there are fewer robots in play, robots at the end of the list will not be
        // assigned
        yield(tactic_vector);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ExamplePlay, PlayConfig> factory;
