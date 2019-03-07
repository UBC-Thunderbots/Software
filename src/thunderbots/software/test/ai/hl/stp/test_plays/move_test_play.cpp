#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/play/play_factory.h"
#include "test/ai/hl/stp/test_tactics/move_test_tactic.h"

/**
 * A test Play that moves a robot to the friendly goal, a robot to the enemy goal, and
 * a robot to the center of the field.
 *
 * The Play is done when the robot reaches the center of the field (within 5cm)
 *
 * This play is applicable when the ball's x and y coordinates are >= 0
 * This play's invariant holds while the ball's x coordinate is >= 0
 *
 * We use the Ball's position to control the 'isApplicable' and 'invariantHolds' values
 * because it is easy to change during tests
 */
class MoveTestPlay : public Play
{
   public:
    MoveTestPlay() : Play(){};

    std::string name() override
    {
        return "Move Test Play";
    }

    bool isApplicable(const World &world) override
    {
        return world.ball().position().x() >= 0;
    }

    bool invariantHolds(const World &world) override
    {
        return world.ball().position().x() >= 0;
    }

    std::vector<std::shared_ptr<Tactic>> getNextTactics(TacticCoroutine::push_type &yield,
                                                        const World &world) override
    {
        auto move_test_tactic_friendly_goal = std::make_shared<MoveTestTactic>();
        auto move_test_tactic_enemy_goal = std::make_shared<MoveTestTactic>();
        auto move_test_tactic_center_field = std::make_shared<MoveTestTactic>();

        do
        {
            move_test_tactic_friendly_goal->updateParams(world.field().friendlyGoal());
            move_test_tactic_enemy_goal->updateParams(world.field().enemyGoal());
            move_test_tactic_center_field->updateParams(Point(0, 0));

            yield({move_test_tactic_center_field, move_test_tactic_friendly_goal, move_test_tactic_enemy_goal});
        } while (!move_test_tactic_center_field->done());
    }
};

// Register this play in the PlayFactory
static TPlayFactory<MoveTestPlay> factory;
