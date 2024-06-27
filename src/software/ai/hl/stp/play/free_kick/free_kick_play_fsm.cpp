#include "free_kick_play_fsm.h"

FreeKickPlayFSM::FreeKickPlayFSM(const TbotsProto::AiConfig& ai_config)
    : ai_config(ai_config),
      best_pass_and_score_so_far(
          PassWithRating{.pass = Pass(Point(), Point(), 0), .rating = 0}),
      align_to_ball_tactic(std::make_shared<MoveTactic>()),
      shoot_tactic(std::make_shared<KickTactic>()),
      chip_tactic(std::make_shared<ChipTactic>()),
      passer_tactic(std::make_shared<KickTactic>()),
      receiver_tactic(std::make_shared<ReceiverTactic>(ai_config.receiver_tactic_config())),
      offensive_positioning_tactics(
          {std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>()}),
      defense_play(std::make_shared<DefensePlay>(ai_config)),
      pass_generator(ai_config.passing_config()),
      receiver_position_generator(ReceiverPositionGenerator<EighteenZoneId>(
              std::make_shared<const EighteenZonePitchDivision>(
                      Field::createSSLDivisionBField()),
              ai_config.passing_config())),
{
}

void FreeKickPlayFSM::setupPosition(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // Passing robot
    updateAlignToBallTactic(event.common.world_ptr);
    tactics_to_run[0].emplace_back(align_to_ball_tactic);

    if (event.common.num_tactics <= 1)
    {
        LOG(WARNING) << "Not enough tactics to setup receivers and crease defenders for free kick";
        return;
    }

    // Up to 2 receivers
    unsigned int num_receivers = std::min(2u, event.common.num_tactics - 1);
    updateOffensivePositioningTactics(event.common.world_ptr, num_receivers);
    tactics_to_run[0].insert(tactics_to_run[0].end(), offensive_positioning_tactics.begin(),
                          offensive_positioning_tactics.end());

    // Crease defenders
    crease_defender_tactics[0]->updateControlParams(
        event.common.world_ptr->ball().position(),
        TbotsProto::CreaseDefenderAlignment::LEFT);
    crease_defender_tactics[1]->updateControlParams(
        event.common.world_ptr->ball().position(),
        TbotsProto::CreaseDefenderAlignment::RIGHT);
    tactics_to_run[0].emplace_back(crease_defender_tactics[0]);
    tactics_to_run[0].emplace_back(crease_defender_tactics[1]);

    event.common.set_tactics(tactics_to_run);
}

bool FreeKickPlayFSM::setupDone(const Update &event)
{
    if (align_to_ball_tactic->done())
    {
        LOG(DEBUG) << "Finished aligning to ball.";
    }
    return align_to_ball_tactic->done();
}

void FreeKickPlayFSM::updateOffensivePositioningTactics(
        const WorldPtr world, unsigned int num_tactics,
        const std::vector<Point>& existing_receiver_positions,
        const std::optional<Point>& pass_origin_override)
{
    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    if (num_tactics != offensive_positioning_tactics.size())
    {
        offensive_positioning_tactics =
                std::vector<std::shared_ptr<MoveTactic>>(num_tactics);
        std::generate(offensive_positioning_tactics.begin(),
                      offensive_positioning_tactics.end(),
                      []() { return std::make_shared<MoveTactic>(); });
    }

    std::vector<Point> best_receiving_positions =
            receiver_position_generator.getBestReceivingPositions(
                    *world, num_tactics, existing_receiver_positions, pass_origin_override);
    // Note that getBestReceivingPositions may return fewer positions than requested
    // if there are not enough robots, so we will need to check the size of the vector.
    for (unsigned int i = 0;
         i < offensive_positioning_tactics.size() && i < best_receiving_positions.size();
         i++)
    {
        Angle receiver_orientation =
                (world->ball().position() - best_receiving_positions[i]).orientation();
        offensive_positioning_tactics[i]->updateControlParams(
                best_receiving_positions[i], receiver_orientation, 0.0,
                TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);
    }
}

void FreeKickPlayFSM::setTactics(const Update& event, int num_receivers,
                                 int num_defenders)
{
    PriorityTacticVector tactics_to_return;

    if (num_receivers > 0)
    {
        // Up to 2 receivers
        unsigned int num_receivers = std::min(2u, event.common.num_tactics - 1);
        updateOffensivePositioningTactics(event.common.world_ptr, num_receivers);
        tactics_to_run[0].insert(tactics_to_run[0].end(), offensive_positioning_tactics.begin(),
                                 offensive_positioning_tactics.end());
    }

    defense_play->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    if (num_defenders > 0)
    {
        defense_play->updateTactics(PlayUpdate(
                event.common.world_ptr, num_defenders,
                [&tactics_to_return](PriorityTacticVector new_tactics) {
                    for (const auto& tactic_vector : new_tactics)
                    {
                        tactics_to_return.push_back(tactic_vector);
                    }
                },
                event.common.inter_play_communication,
                event.common.set_inter_play_communication_fun));
    }

    event.common.set_tactics(tactics_to_return);
}

void FreeKickPlayFSM::updateAlignToBallTactic(const WorldPtr &world_ptr)
{
    // We want the kicker to get into position behind the ball facing the enemy net
    Point ball_pos                = world_ptr->ball().position();

    std::optional<Robot> robot_kicking_opt = world_ptr->friendlyTeam().getNearestRobot(ball_pos);
    if (!robot_kicking_opt.has_value())
    {
        return;
    }

    // Face towards the friendly farthest up the field
    Point position_to_face(-1, 0);
    for (const Robot& friendly: world_ptr->friendlyTeam().getAllRobots())
    {
        // Skip over the robot kick the ball
        if (friendly.id() == robot_kicking_opt->id())
        {
            continue;
        }
        // TODO (NIMA): Should face best receiving robot?!?
        if (friendly.position().x() > position_to_face.x())
        {
            position_to_face = friendly.position();
        }
    }
    Vector direction_to_face = position_to_face - ball_pos;
    align_to_ball_tactic->updateControlParams(
        ball_pos - direction_to_face.normalize(ROBOT_MAX_RADIUS_METERS * 2),
        direction_to_face.orientation(), 0);
}

bool FreeKickPlayFSM::shotFound(const Update &event)
{
    shot = calcBestShotOnGoal(event.common.world_ptr->field(),
                              event.common.world_ptr->friendlyTeam(),
                              event.common.world_ptr->enemyTeam(),
                              event.common.world_ptr->ball().position(), TeamType::ENEMY);
    return shot.has_value() &&
           shot->getOpenAngle() >
               Angle::fromDegrees(
                   ai_config.attacker_tactic_config().min_open_angle_for_shot_deg());
}

void FreeKickPlayFSM::shootBall(const Update &event)
{
    LOG(DEBUG) << "Shooting ball...";
    PriorityTacticVector tactics_to_run = {{}};

    Point ball_pos = event.common.world_ptr->ball().position();

    shoot_tactic->updateControlParams(
        ball_pos, (shot->getPointToShootAt() - ball_pos).orientation(),
        BALL_MAX_SPEED_METERS_PER_SECOND);
    tactics_to_run[0].emplace_back(shoot_tactic);

    event.common.set_tactics(tactics_to_run);
}

void FreeKickPlayFSM::startLookingForPass(const FreeKickPlayFSM::Update &event)
{
    pass_optimization_start_time = event.common.world_ptr->getMostRecentTimestamp();
}

bool FreeKickPlayFSM::timeExpired(const FreeKickPlayFSM::Update &event)
{
    Duration time_since_pass_optimization_start =
        event.common.world_ptr->getMostRecentTimestamp() - pass_optimization_start_time;
    return time_since_pass_optimization_start.toSeconds() >
           ai_config.free_kick_play_config().max_time_commit_to_pass_seconds();
}

void FreeKickPlayFSM::chipBall(const Update &event)
{
    LOG(DEBUG) << "Time to look for pass expired. Chipping ball...";
    PriorityTacticVector tactics_to_run = {{}};

    Point ball_pos = event.common.world_ptr->ball().position();
    std::optional<Robot> robot_kicking_opt = event.common.world_ptr->friendlyTeam().getNearestRobot(ball_pos);
    if (!robot_kicking_opt.has_value())
    {
        return;
    }

    // Chip towards the friendly farthest up the enemy half
    Point chip_target(0, 0);
    for (const Robot& friendly: event.common.world_ptr->friendlyTeam().getAllRobots())
    {
        // Skip over the robot kick the ball
        if (friendly.id() == robot_kicking_opt->id())
        {
            continue;
        }
        // TODO (NIMA): Should face best receiving robot?!?
        if (friendly.position().x() > chip_target.x())
        {
            chip_target = friendly.position();
        }
    }

    chip_tactic->updateControlParams(event.common.world_ptr->ball().position(),
                                     chip_target);
    tactics_to_run[0].emplace_back(chip_tactic);

    event.common.set_tactics(tactics_to_run);
}

void FreeKickPlayFSM::lookForPass(const FreeKickPlayFSM::Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // Keep the kicker aligned to the ball
    updateAlignToBallTactic(event.common.world_ptr);
    tactics_to_run[0].emplace_back(align_to_ball_tactic);

    // Up to 2 receivers
    unsigned int num_receivers = std::min(2u, event.common.num_tactics - 1);
    updateOffensivePositioningTactics(event.common.world_ptr, num_receivers);
    tactics_to_run[0].insert(tactics_to_run[0].end(), offensive_positioning_tactics.begin(),
                             offensive_positioning_tactics.end());

    // Maintain crease defenders
    crease_defender_tactics[0]->updateControlParams(
        event.common.world_ptr->ball().position(),
        TbotsProto::CreaseDefenderAlignment::LEFT);
    crease_defender_tactics[1]->updateControlParams(
        event.common.world_ptr->ball().position(),
        TbotsProto::CreaseDefenderAlignment::RIGHT);
    tactics_to_run[0].emplace_back(crease_defender_tactics[0]);
    tactics_to_run[0].emplace_back(crease_defender_tactics[1]);

    // Avoid passes to the goalie and the passing robot
    std::vector<RobotId> robots_to_ignore = {};
    auto friendly_goalie_id_opt =
            event.common.world_ptr->friendlyTeam().getGoalieId();
    if (friendly_goalie_id_opt.has_value())
    {
        robots_to_ignore.push_back(friendly_goalie_id_opt.value());
    }
    auto robot_with_ball_opt = event.common.world_ptr->friendlyTeam().getNearestRobot(
            event.common.world_ptr->ball().position());
    if (robot_with_ball_opt.has_value())
    {
        robots_to_ignore.push_back(robot_with_ball_opt.value().id());
    }
    best_pass_and_score_so_far =
        pass_generator.getBestPass(*event.common.world_ptr, robots_to_ignore);

    event.common.set_tactics(tactics_to_run);
}

bool FreeKickPlayFSM::passFound(const Update &event)
{
    double time_since_pass_optimization_start_seconds =
        (event.common.world_ptr->getMostRecentTimestamp() - pass_optimization_start_time)
            .toSeconds();

    double abs_min_pass_score =
            ai_config.shoot_or_pass_play_config().abs_min_pass_score();
    double min_perfect_pass_score =
            ai_config.shoot_or_pass_play_config().min_perfect_pass_score();
    double pass_score_ramp_down_duration =
            ai_config.free_kick_play_config().max_time_commit_to_pass_seconds();

    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    double min_score = min_perfect_pass_score -
                               std::min(time_since_pass_optimization_start_seconds /
                                        pass_score_ramp_down_duration,
                                        min_perfect_pass_score - abs_min_pass_score);
    LOG(DEBUG) << "Pass Score: " << best_pass_and_score_so_far.rating
               << " Score threshold: " << min_score;

    return best_pass_and_score_so_far.rating > min_score;
}

bool FreeKickPlayFSM::shouldAbortPass(const Update& event)
{
    // Check if ball has already been passed
    if (distance(event.common.world_ptr->ball().position(), best_pass_and_score_so_far.pass.passerPoint()) > 0.05)
    {
        return false;
    }

    // Abort pass if the pass score has dropped significantly
    best_pass_and_score_so_far.rating =
            ratePass(*event.common.world_ptr, best_pass_and_score_so_far.pass,
                     ai_config.passing_config());
    double abs_min_pass_score =
            ai_config.shoot_or_pass_play_config().abs_min_pass_score();
    return best_pass_and_score_so_far.rating < abs_min_pass_score;
}


void FreeKickPlayFSM::passBall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // We have committed to the pass
    LOG(DEBUG) << "Found pass with score: " << best_pass_and_score_so_far.rating
               << ". Passing...";

    Pass pass = best_pass_and_score_so_far.pass;

    passer_tactic->updateControlParams(pass.passerPoint(), pass.passerOrientation(),
                                        pass.speed());
    receiver_tactic->updateControlParams(pass);
    tactics_to_run[0].emplace_back(passer_tactic);
    tactics_to_run[0].emplace_back(receiver_tactic);

    // Up to 1 receiver
    unsigned int num_receivers = std::min(1u, event.common.num_tactics - 1);
    updateOffensivePositioningTactics(event.common.world_ptr, num_receivers, {pass.receiverPoint()}, pass.receiverPoint());
    tactics_to_run[0].insert(tactics_to_run[0].end(), offensive_positioning_tactics.begin(),
                             offensive_positioning_tactics.end());

    // Maintain crease defenders
    crease_defender_tactics[0]->updateControlParams(
            event.common.world_ptr->ball().position(),
            TbotsProto::CreaseDefenderAlignment::LEFT);
    crease_defender_tactics[1]->updateControlParams(
            event.common.world_ptr->ball().position(),
            TbotsProto::CreaseDefenderAlignment::RIGHT);
    tactics_to_run[0].emplace_back(crease_defender_tactics[0]);
    tactics_to_run[0].emplace_back(crease_defender_tactics[1]);

    event.common.set_tactics(tactics_to_run);
}

bool FreeKickPlayFSM::shotDone(const Update &event)
{
    if (shoot_tactic->done())
    {
        LOG(DEBUG) << "Finished shot.";
    }
    return shoot_tactic->done();
}

bool FreeKickPlayFSM::passDone(const FreeKickPlayFSM::Update &event)
{
    if (receiver_tactic->done())
    {
        LOG(DEBUG) << "Finished pass.";
    }
    return receiver_tactic->done();
}

bool FreeKickPlayFSM::chipDone(const FreeKickPlayFSM::Update &event)
{
    if (chip_tactic->done())
    {
        LOG(DEBUG) << "Finished chip.";
    }
    return chip_tactic->done();
}
