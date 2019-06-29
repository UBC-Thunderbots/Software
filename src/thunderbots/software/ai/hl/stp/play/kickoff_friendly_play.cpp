#include "ai/hl/stp/play/kickoff_friendly_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/chip_tactic.h"
#include "ai/hl/stp/tactic/goalie_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "shared/constants.h"

const std::string KickoffFriendlyPlay::name = "KickoffFriendly Play";

std::string KickoffFriendlyPlay::getName() const
{
    return KickoffFriendlyPlay::name;
}

bool KickoffFriendlyPlay::isApplicable(const World &world) const
{
    return (world.gameState().isReadyState() || world.gameState().isSetupState()) &&
           world.gameState().isOurKickoff();
}

bool KickoffFriendlyPlay::invariantHolds(const World &world) const
{
    return !world.gameState().isPlaying();
}

void KickoffFriendlyPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
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
    //      Part 1: Get into position, but don't touch the ball (ref kickoff)
    //      Part 2: Chip the ball over the defender (ref normal start)

    // the following positions are in the same order as the positions shown above,
    // excluding the goalie for part 1 of this play
    std::vector<Point> kickoff_setup_positions = {
        // Robot 1
        Point(world.field().centerPoint() +
              Point(-world.field().centreCircleRadius(), 0)),
        // Robot 2
        // Goalie positions will be handled by the goalie tactic
        // Robot 3
        Point(world.field().centerPoint() +
              Point(-world.field().centreCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                    -1.0 / 3.0 * world.field().width())),
        // Robot 4
        Point(world.field().centerPoint() +
              Point(-world.field().centreCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                    1.0 / 3.0 * world.field().width())),
        // Robot 5
        Point(world.field().friendlyGoalpostPos().x() +
                  world.field().defenseAreaLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().friendlyGoalpostPos().y()),
        // Robot 6
        Point(world.field().friendlyGoalpostNeg().x() +
                  world.field().defenseAreaLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().friendlyGoalpostNeg().y()),
    };

    // move tactics to use to move to positions defined above
    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true)};

    // specific tactics
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
    auto chip_tactic = std::make_shared<ChipTactic>(world.ball(), true);

    // Part 1: setup state (move to key positions)
    while (world.gameState().isSetupState())
    {
        auto enemy_threats = Evaluation::getAllEnemyThreats(
            world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);

        goalie_tactic->updateParams(
            world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam(),
            enemy_threats.empty() ? std::nullopt
                                  : std::make_optional(enemy_threats.at(0)));
        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic};

        // set the requirement that Robot 1 must be able to kick and chip
        move_tactics.at(0)->mutableRobotCapabilityRequirements() = {
            RobotCapabilityFlags::Kick, RobotCapabilityFlags::Chip};

        // setup 5 kickoff positions in order of priority
        for (int i = 0; i < kickoff_setup_positions.size(); i++)
        {
            move_tactics.at(i)->updateParams(kickoff_setup_positions.at(i), Angle::half(),
                                             0);
            result.emplace_back(move_tactics.at(i));
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    }

    // Part 2: not normal play, currently ready state (chip the ball)
    while (!world.gameState().isPlaying())
    {
        auto enemy_threats = Evaluation::getAllEnemyThreats(
            world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);

        goalie_tactic->updateParams(
            world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam(),
            enemy_threats.empty() ? std::nullopt
                                  : std::make_optional(enemy_threats.at(0)));
        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic};

        // TODO This needs to be adjusted post field testing, ball needs to land exactly
        // in the middle of the enemy field
        chip_tactic->updateParams(
            world.ball(), world.field().centerPoint(),
            world.field().centerPoint() + Point(world.field().length() / 4, 0),
            world.field().length() / 2);
        result.emplace_back(chip_tactic);

        // the robot at position 0 will be closest to the ball, so positions starting from
        // 1 will be assigned to the rest of the robots
        for (int i = 1; i < kickoff_setup_positions.size(); i++)
        {
            move_tactics.at(i)->updateParams(kickoff_setup_positions.at(i), Angle::half(),
                                             0);
            result.emplace_back(move_tactics.at(i));
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    }
}

// Register this play in the PlayFactory
static TPlayFactory<KickoffFriendlyPlay> factory;
