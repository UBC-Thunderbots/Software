#include "software/ai/hl/stp/play/kickoff_friendly_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

KickoffFriendlyPlay::KickoffFriendlyPlay(std::shared_ptr<const AiConfig> config)
    : Play(config, true)
{
}

void KickoffFriendlyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                         const World &world)
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
              Vector(-world.field().centerCircleRadius(), 0)),
        // Robot 2
        // Goalie positions will be handled by the goalie tactic
        // Robot 3
        Point(world.field().centerPoint() +
              Vector(-world.field().centerCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                     -1.0 / 3.0 * world.field().yLength())),
        // Robot 4
        Point(world.field().centerPoint() +
              Vector(-world.field().centerCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                     1.0 / 3.0 * world.field().yLength())),
        // Robot 5
        Point(world.field().friendlyGoalpostPos().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().friendlyGoalpostPos().y()),
        // Robot 6
        Point(world.field().friendlyGoalpostNeg().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().friendlyGoalpostNeg().y()),
    };

    // move tactics to use to move to positions defined above
    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>()};

    // specific tactics
    auto kickoff_chip_tactic = std::make_shared<KickoffChipTactic>();

    // Part 1: setup state (move to key positions)
    while (world.gameState().isSetupState())
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        PriorityTacticVector result = {{}};

        // set the requirement that Robot 1 must be able to kick and chip
        move_tactics.at(0)->mutableRobotCapabilityRequirements() = {
            RobotCapability::Kick, RobotCapability::Chip};

        // setup 5 kickoff positions in order of priority
        for (unsigned i = 0; i < kickoff_setup_positions.size(); i++)
        {
            move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                    Angle::zero(), 0);
            result[0].emplace_back(move_tactics.at(i));
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    }

    // Part 2: not normal play, currently ready state (chip the ball)
    while (!world.gameState().isPlaying())
    {
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        PriorityTacticVector result = {{}};

        // TODO This needs to be adjusted post field testing, ball needs to land exactly
        // in the middle of the enemy field
        kickoff_chip_tactic->updateControlParams(
            world.ball().position(),
            world.field().centerPoint() + Vector(world.field().xLength() / 6, 0));
        result[0].emplace_back(kickoff_chip_tactic);

        // the robot at position 0 will be closest to the ball, so positions starting from
        // 1 will be assigned to the rest of the robots
        for (unsigned i = 1; i < kickoff_setup_positions.size(); i++)
        {
            move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                    Angle::zero(), 0);
            result[0].emplace_back(move_tactics.at(i));
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    }
}


// Register this play in the genericFactory
static TGenericFactory<std::string, Play, KickoffFriendlyPlay, AiConfig> factory;
