#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/play/play_factory.h"
#include "geom/util.h"
#include "test/ai/hl/stp/test_tactics/stop_test_tactic.h"

/**
 * A test Play that stops 3 robots.
 *
 * The Play is never done
 *
 * This play is applicable when the ball's y coordinate is >= 0
 * This play's invariant holds while the ball is within the field
 *
 * We use the Ball's position to control the 'isApplicable' and 'invariantHolds' values
 * because it is easy to change during tests
 */
class StopTestPlay : public Play
{
   public:
    StopTestPlay() : Play(){};

    std::string name() override
    {
        return "Stop Test Play";
    }

    bool isApplicable(const World &world) override
    {
        return world.ball().position().y() >= 0;
    }

    bool invariantHolds(const World &world) override
    {
        return contains(
            Rectangle(world.field().enemyCornerNeg(), world.field().friendlyCornerPos()),
            world.ball().position());
    }

    std::vector<std::shared_ptr<Tactic>> getNextTactics(TacticCoroutine::push_type &yield,
                                                        const World &world) override
    {
        auto stop_test_tactic_1 = std::make_shared<StopTestTactic>();
        auto stop_test_tactic_2 = std::make_shared<StopTestTactic>();
        auto stop_test_tactic_3 = std::make_shared<StopTestTactic>();

        do
        {
            stop_test_tactic_1->updateParams();
            stop_test_tactic_2->updateParams();
            stop_test_tactic_3->updateParams();

            yield({stop_test_tactic_1, stop_test_tactic_2, stop_test_tactic_3});
        } while (true);
    }
};

// Register this play in the PlayFactory
static TPlayFactory<StopTestPlay> factory;
