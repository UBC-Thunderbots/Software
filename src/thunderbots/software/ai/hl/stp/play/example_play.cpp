#include "ai/hl/stp/play/example_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/movespin_tactic.h"

const std::string ExamplePlay::name = "Example Play";

std::string ExamplePlay::getName() const
{
    return ExamplePlay::name;
}

bool ExamplePlay::isApplicable(const World &world) const
{
    return true;
}

bool ExamplePlay::invariantHolds(const World &world) const
{
    return true;
}

void ExamplePlay::getNextTactics(TacticCoroutine::push_type &yield)
{

    // Create MoveSpinTactics that will loop forever
    auto movespin_tactic_1 = std::make_shared<MoveSpinTactic>(true);
    auto movespin_tactic_2 = std::make_shared<MoveSpinTactic>(true);


    for (int i = 0;i<3;i++)
    {
        // The angle between each robot spaced out in a circle around the ball
        Angle angle_between_robots = Angle::full() / world.friendlyTeam().numRobots();

        // Move the robots in a circle around the ball, facing the ball

        // yield the Tactics this Play wants to run, in order of priority
        yield({movespin_tactic_1});
        std::cout<<i+100000;
    }

}

// Register this play in the PlayFactory
static TPlayFactory<ExamplePlay> factory;
