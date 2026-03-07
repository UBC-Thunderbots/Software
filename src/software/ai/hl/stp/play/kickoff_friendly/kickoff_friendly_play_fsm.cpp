#include "software/ai/hl/stp/play/kickoff_friendly/kickoff_friendly_play_fsm.h"

KickoffFriendlyPlayFSM::KickoffFriendlyPlayFSM(
    const std::shared_ptr<const TbotsProto::AiConfig>& ai_config_ptr)
    : PlayFSM<KickoffFriendlyPlayFSM>(ai_config_ptr),
      kickoff_chip_tactic(std::make_shared<KickoffChipTactic>(ai_config_ptr)),
      move_tactics{std::make_shared<PrepareKickoffMoveTactic>(ai_config_ptr),  // robot 1
                   std::make_shared<MoveTactic>(ai_config_ptr),  // robot 2-5
                   std::make_shared<MoveTactic>(ai_config_ptr),
                   std::make_shared<MoveTactic>(ai_config_ptr),
                   std::make_shared<MoveTactic>(ai_config_ptr)}
{
}

void KickoffFriendlyPlayFSM::createKickoffSetupPositions(const WorldPtr& world_ptr)
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

    if (kickoff_setup_positions.empty())
    {
        kickoff_setup_positions = {
            // Robot 1
            Point(world_ptr->field().centerPoint() +
                  Vector(-world_ptr->field().centerCircleRadius(), 0)),
            // Robot 2
            // Goalie positions will be handled by the goalie tactic
            // Robot 3
            Point(world_ptr->field().centerPoint() +
                  Vector(-world_ptr->field().centerCircleRadius() -
                             4 * ROBOT_MAX_RADIUS_METERS,
                         -1.0 / 3.0 * world_ptr->field().yLength())),
            // Robot 4
            Point(world_ptr->field().centerPoint() +
                  Vector(-world_ptr->field().centerCircleRadius() -
                             4 * ROBOT_MAX_RADIUS_METERS,
                         1.0 / 3.0 * world_ptr->field().yLength())),
            // Robot 5
            Point(world_ptr->field().friendlyGoalpostPos().x() +
                      world_ptr->field().defenseAreaXLength() +
                      2 * ROBOT_MAX_RADIUS_METERS,
                  world_ptr->field().friendlyGoalpostPos().y()),
            // Robot 6
            Point(world_ptr->field().friendlyGoalpostNeg().x() +
                      world_ptr->field().defenseAreaXLength() +
                      2 * ROBOT_MAX_RADIUS_METERS,
                  world_ptr->field().friendlyGoalpostNeg().y()),
        };
    }
}


void KickoffFriendlyPlayFSM::setupKickoff(const Update& event)
{
    createKickoffSetupPositions(event.common.world_ptr);

    PriorityTacticVector tactics_to_run = {{}};

    // first priority requires the ability to kick and chip.
    move_tactics.at(0)->mutableRobotCapabilityRequirements() = {RobotCapability::Kick,
                                                                RobotCapability::Chip};

    // set each tactic to its movement location.
    for (unsigned i = 0; i < kickoff_setup_positions.size(); i++)
    {
        move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                Angle::zero());
        tactics_to_run[0].emplace_back(move_tactics.at(i));
    }

    event.common.set_tactics(tactics_to_run);
}


void KickoffFriendlyPlayFSM::chipBall(const Update& event)
{
    const WorldPtr& world_ptr = event.common.world_ptr;
    const auto& field         = world_ptr->field();
    const Point ball_position = world_ptr->ball().position();

    PriorityTacticVector tactics_to_run = {{}};

    constexpr double enemy_x_padding_m          = 2.0;
    constexpr double sideline_padding_m         = 0.3;
    constexpr double fallback_target_x_fraction = 1.0 / 6.0;

    const double min_chip_x = ball_position.x();
    const double max_chip_x = field.enemyGoalCenter().x() - enemy_x_padding_m;
    const double min_chip_y = field.enemyCornerNeg().y() + sideline_padding_m;
    const double max_chip_y = field.enemyCornerPos().y() - sideline_padding_m;

    const Rectangle chip_target_region(Point(min_chip_x, min_chip_y),
                                       Point(max_chip_x, max_chip_y));
    const Point fallback_target =
        field.centerPoint() + Vector(field.xLength() * fallback_target_x_fraction, 0.0);
    const std::vector<Circle> chip_targets =
        findGoodChipTargets(*world_ptr, chip_target_region);

    Point selected_target = fallback_target;

    if (!chip_targets.empty())
    {
        const auto best_target_it =
            std::min_element(chip_targets.begin(), chip_targets.end(),
                             [&field](const Circle& a, const Circle& b)
                             {
                                 return distance(field.enemyGoalCenter(), a.origin()) <
                                        distance(field.enemyGoalCenter(), b.origin());
                             });

        selected_target = best_target_it->origin();
    }

    kickoff_chip_tactic->updateControlParams(ball_position, selected_target);
    tactics_to_run[0].emplace_back(kickoff_chip_tactic);
    event.common.set_tactics(tactics_to_run);
}


bool KickoffFriendlyPlayFSM::isSetupDone(const Update& event)
{
    return !event.common.world_ptr->gameState().isSetupState();
}

bool KickoffFriendlyPlayFSM::isPlaying(const Update& event)
{
    return event.common.world_ptr->gameState().isPlaying();
}
