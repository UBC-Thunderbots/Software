#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"

/**
 * An example Play that moves the robots in a circle around the ball
 */
class ExamplePlay : public Play
{
   public:
    ExamplePlay() : Play(){};

    std::string name() override
    {
        return "Example Play";
    }

    bool isApplicable(const World &world) override
    {
        return true;
    }

    bool invariantHolds(const World &world) override
    {
        return true;
    }

    std::vector<std::shared_ptr<Tactic>> getNextTactics(TacticCoroutine::push_type &yield,
                                                        const World &world) override
    {
        // Create MoveTactics that will loop forever
        auto move_tactic_1 = std::make_shared<MoveTactic>(true);
        auto move_tactic_2 = std::make_shared<MoveTactic>(true);
        auto move_tactic_3 = std::make_shared<MoveTactic>(true);
        auto move_tactic_4 = std::make_shared<MoveTactic>(true);
        auto move_tactic_5 = std::make_shared<MoveTactic>(true);
        auto move_tactic_6 = std::make_shared<MoveTactic>(true);

        do
        {
            // The angle between each robot spaced out in a circle around the ball
            Angle angle_between_robots = Angle::full() / world.friendlyTeam().numRobots();

            // Move the robots in a circle around the ball, facing the ball
            move_tactic_1->updateParams(
                world.ball().position() +
                    Point::createFromAngle(angle_between_robots * 1),
                (angle_between_robots * 1) + Angle::half(), 0);
            move_tactic_2->updateParams(
                world.ball().position() +
                    Point::createFromAngle(angle_between_robots * 2),
                (angle_between_robots * 2) + Angle::half(), 0);
            move_tactic_3->updateParams(
                world.ball().position() +
                    Point::createFromAngle(angle_between_robots * 3),
                (angle_between_robots * 3) + Angle::half(), 0);
            move_tactic_4->updateParams(
                world.ball().position() +
                    Point::createFromAngle(angle_between_robots * 4),
                (angle_between_robots * 4) + Angle::half(), 0);
            move_tactic_5->updateParams(
                world.ball().position() +
                    Point::createFromAngle(angle_between_robots * 5),
                (angle_between_robots * 5) + Angle::half(), 0);
            move_tactic_6->updateParams(
                world.ball().position() +
                    Point::createFromAngle(angle_between_robots * 6),
                (angle_between_robots * 6) + Angle::half(), 0);

            // yield the Tactics this Play wants to run, in order of priority
            yield({move_tactic_1, move_tactic_2, move_tactic_3, move_tactic_4,
                   move_tactic_5, move_tactic_6});
        } while (true);
    }
};

// Register this play in the PlayFactory
static TPlayFactory<ExamplePlay> factory;
