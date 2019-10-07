#include "software/ai/hl/stp/play/example_play.h"

#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"

const std::string ExamplePlay::name = "Example Play";

/**
 * This function returns name of the play
 * @return ExamplePlay's name (which is "Example Play" as seen on Line 6)
 */
std::string ExamplePlay::getName() const
{
    return ExamplePlay::name;
}

/**
 * This function returns whether the play is applicable
 * Applicable describes the conditions for the play should start
 * For instance, the play should start when it is kick-off
 * @return If ExamplePlay is applicable
 * It is applicable since it should start regardless of anything else
 * since it is a demo
 */
bool ExamplePlay::isApplicable(const World &world) const
{
    return true;
}

/**
 * This function returns whether the play's invariant holds
 * Invariant describes the conditions for the play to keep running
 * For instance, the invariant may not hold if we lose possession of the ball
 * @return If ExamplePlay's invariant holds
 * It holds true since it should keep running regardless of anything else
 * since it is a demo
 */
bool ExamplePlay::invariantHolds(const World &world) const
{
    return true;
}

void ExamplePlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Create MoveTactics that will loop forever
    auto move_tactic_1 = std::make_shared<MoveTactic>(true);
    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);

    // Continue to loop to demonstrate the example play indefinitely
    do
    {
        // The angle between each robot spaced out in a circle around the ball
        Angle angle_between_robots = Angle::full() / world.friendlyTeam().numRobots();

        // Move the robots in a circle around the ball, facing the ball
        move_tactic_1->updateControlParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 1),
            (angle_between_robots * 1) + Angle::half(), 0);
        move_tactic_2->updateControlParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 2),
            (angle_between_robots * 2) + Angle::half(), 0);
        move_tactic_3->updateControlParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 3),
            (angle_between_robots * 3) + Angle::half(), 0);
        move_tactic_4->updateControlParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 4),
            (angle_between_robots * 4) + Angle::half(), 0);
        move_tactic_5->updateControlParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 5),
            (angle_between_robots * 5) + Angle::half(), 0);
        move_tactic_6->updateControlParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 6),
            (angle_between_robots * 6) + Angle::half(), 0);

        // yield the Tactics this Play wants to run, in order of priority
        // If there are fewer robots in play, robots at the end of the list will not be assigned
        yield({move_tactic_1, move_tactic_2, move_tactic_3, move_tactic_4, move_tactic_5,
               move_tactic_6});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<ExamplePlay> factory;
