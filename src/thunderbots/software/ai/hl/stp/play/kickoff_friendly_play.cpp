#include "ai/hl/stp/play/kickoff_friendly_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "shared/constants.h"

const std::string KickoffFriendlyPlay::name = "KickoffFriendly Play";

std::string KickoffFriendlyPlay::getName() const
{
    return KickoffFriendlyPlay::name;
}

bool KickoffFriendlyPlay::isApplicable(const World &world) const
{
    return (world.gameState().isReadyState() | world.gameState().isSetupState()) &&
        world.gameState().isOurKickoff();
}

bool KickoffFriendlyPlay::invariantHolds(const World &world) const
{
    return !world.gameState().isPlaying();
}

void KickoffFriendlyPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // TODO: This needs to be a goalie tactic
    auto lone_goalie_tactic_0 = std::make_shared<MoveTactic>(true);

    // Since we only have 6 robots at the maximum, the number one priority
    // is the robot doing the kickoff up front. The goalie is the second most
    // important, followed by 3 and 4 setup for offense. 5 and 6 will stay
    // back near the goalie just in case the ball quickly returns to the friendly
    // side of the field.
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|               3    |                    |
    // 		|                    |                    |
    // 		+--+ 5               |                 +--+
    // 		|  |                 |                 |  |
    // 		|  |               +-+-+               |  |
    // 		|2 |               |1  |               |  |
    // 		|  |               +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+ 6               |                 +--+
    // 		|                    |                    |
    // 		|               4    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+
    //
    // This is a two part play:
    //      Part 1: Get into position for the kickoff, but don't touch the ball (ref kickoff)
    //      Part 2: Chip the ball over the defender (ref normal start)

    // the following positions are in the same order as the positions shown above,
    // excluding the goalie for part 1 of this play
    std::vector<Point> kickoff_setup_positions = {
        // Robot 1
        Point(world.field().centerPoint() + Point(world.field().centreCircleRadius(), 0)),
        // Robot 2
        // Goalie positions will be handled by the goalie tactic
        // Robot 3
        Point(world.field().centerPoint() +
                Point(-world.field().centreCircleRadius() - 4*ROBOT_MAX_RADIUS_METERS, -2/3*world.field().width())),
        // Robot 4
        Point(world.field().centerPoint() +
                Point(-world.field().centreCircleRadius() - 4*ROBOT_MAX_RADIUS_METERS, +2/3*world.field().width())),
        // Robot 5
        Point(world.field().friendlyGoalpostPos().x() +
                world.field().defenseAreaLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                world.field().friendlyGoalpostPos().y()),
        // Robot 6
        Point(world.field().friendlyGoalpostNeg().x() +
                world.field().defenseAreaLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                world.field().friendlyGoalpostNeg().y()),
    };

    // move to setup positions
    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true)};

    do
    {
        // TODO: Replace placeholder tactic with goalie tactic
        lone_goalie_tactic_0->updateParams(
                world.field().friendlyGoal(),
                (world.ball().position() - world.field().friendlyGoal()).orientation(), 0);

        std::vector<std::shared_ptr<Tactic>> result = {lone_goalie_tactic_0};

        // assign the remaining robots to the remaining move tactics. If 3 robots were
        // properly assigned to the top 3 threats, then only the first 2 defense positions
        // will be assigned
        for (int i = 0; i < kickoff_setup_positions.size(); i++)
        {
            move_tactics.at(i)
                ->updateParams(kickoff_setup_positions.at(i),
                        Angle::half(), 0);
            result.emplace_back(move_tactics.at(i));
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<KickoffFriendlyPlay> factory;
