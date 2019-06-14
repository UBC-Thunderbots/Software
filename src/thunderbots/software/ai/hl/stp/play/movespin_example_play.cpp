#include "ai/hl/stp/play/movespin_example_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/movespin_tactic.h"

const std::string MoveSpinExamplePlay::name = "MoveSpin Example Play";

std::string MoveSpinExamplePlay::getName() const
{
    return MoveSpinExamplePlay::name;
}

bool MoveSpinExamplePlay::isApplicable(const World &world) const
{
    return true;
}

bool MoveSpinExamplePlay::invariantHolds(const World &world) const
{
    return true;
}

void MoveSpinExamplePlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Create MoveSpinTactics that will loop forever
    auto movespin_tactic_1 = std::make_shared<MoveSpinTactic>(true);
    auto movespin_tactic_2 = std::make_shared<MoveSpinTactic>(true);


    do
    {
        // The angle between each robot spaced out in a circle around the ball
        Angle angle_between_robots = Angle::full() / world.friendlyTeam().numRobots();

        // Move the robots in a circle around the ball, facing the ball
        movespin_tactic_1->updateParams(
                world.ball().position() + Point::createFromAngle(angle_between_robots * 1),
                (angle_between_robots * 1) + Angle::half(), 0);
        movespin_tactic_2->updateParams(
                world.ball().position() + Point::createFromAngle(angle_between_robots * 2),
                (angle_between_robots * 2) + Angle::half(), 2);

        // yield the Tactics this Play wants to run, in order of priority
        yield({movespin_tactic_1, movespin_tactic_2});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<MoveSpinExamplePlay> factory;

