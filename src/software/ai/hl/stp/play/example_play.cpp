#include "software/ai/hl/stp/play/example_play.h"

#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

ExamplePlay::ExamplePlay(std::shared_ptr<const PlayConfig> config)
{
    play_config = config;
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
        Angle angle_between_robots =
            Angle::full() / static_cast<double>(world.friendlyTeam().numRobots());

        // Move the robots in a circle around the ball, facing the ball
        move_tactic_1->updateControlParams(
            world.ball().position() + Vector::createFromAngle(angle_between_robots * 1),
            (angle_between_robots * 1) + Angle::half(), 0);
        move_tactic_2->updateControlParams(
            world.ball().position() + Vector::createFromAngle(angle_between_robots * 2),
            (angle_between_robots * 2) + Angle::half(), 0);
        move_tactic_3->updateControlParams(
            world.ball().position() + Vector::createFromAngle(angle_between_robots * 3),
            (angle_between_robots * 3) + Angle::half(), 0);
        move_tactic_4->updateControlParams(
            world.ball().position() + Vector::createFromAngle(angle_between_robots * 4),
            (angle_between_robots * 4) + Angle::half(), 0);
        move_tactic_5->updateControlParams(
            world.ball().position() + Vector::createFromAngle(angle_between_robots * 5),
            (angle_between_robots * 5) + Angle::half(), 0);
        move_tactic_6->updateControlParams(
            world.ball().position() + Vector::createFromAngle(angle_between_robots * 6),
            (angle_between_robots * 6) + Angle::half(), 0);

        // yield the Tactics this Play wants to run, in order of priority
        // If there are fewer robots in play, robots at the end of the list will not be
        // assigned
        yield({move_tactic_1, move_tactic_2, move_tactic_3, move_tactic_4, move_tactic_5,
               move_tactic_6});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ExamplePlay, PlayConfig> factory;
