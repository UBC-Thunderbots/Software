#include "software/ai/hl/stp/play/free_kick/free_kick_play_fsm.h"

FreeKickPlayFSM::FreeKickPlayFSM(std::shared_ptr<Strategy> strategy)
    : strategy(strategy),
      align_to_ball_tactic(std::make_shared<MoveTactic>()),
      shoot_tactic(std::make_shared<AssignedSkillTactic<KickSkill>>(strategy)),
      chip_tactic(std::make_shared<AssignedSkillTactic<ChipSkill>>(strategy)),
      passer_tactic(std::make_shared<AssignedSkillTactic<KickSkill>>(strategy)),
      receiver_tactic(std::make_shared<ReceiverTactic>(strategy)),
      receiver_positioning_tactics(
          {std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>()}),
      defense_play(std::make_shared<DefensePlay>(strategy)),
      best_pass_and_score_so_far(
          PassWithRating{.pass = Pass(Point(), Point(), 0), .rating = 0})
{
}

void FreeKickPlayFSM::setupPosition(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // Passing robot
    updateAlignToBallTactic(event.common.world_ptr);
    tactics_to_run[0].emplace_back(align_to_ball_tactic);

    // Already assigned one tactic to align to ball.
    // Assign up to 2 receivers and the remaining tactics are defenders
    setReceiverAndDefenderTactics(tactics_to_run, event, 2, 1);

    event.common.set_tactics(tactics_to_run);
}

bool FreeKickPlayFSM::setupDone(const Update &event)
{
    return align_to_ball_tactic->done();
}

void FreeKickPlayFSM::updateReceiverPositioningTactics(
    const WorldPtr world, unsigned int num_tactics,
    const std::vector<Point> &existing_receiver_positions,
    const std::optional<Point> &pass_origin_override)
{
    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    if (num_tactics != receiver_positioning_tactics.size())
    {
        receiver_positioning_tactics =
            std::vector<std::shared_ptr<MoveTactic>>(num_tactics);
        std::generate(receiver_positioning_tactics.begin(),
                      receiver_positioning_tactics.end(),
                      []() { return std::make_shared<MoveTactic>(); });
    }

    std::vector<Point> best_receiving_positions = strategy->getBestReceivingPositions(
        num_tactics, existing_receiver_positions, pass_origin_override);

    // Note that getBestReceivingPositions may return fewer positions than requested
    // if there are not enough robots, so we will need to check the size of the vector.
    for (unsigned int i = 0;
         i < receiver_positioning_tactics.size() && i < best_receiving_positions.size();
         i++)
    {
        Angle receiver_orientation =
            (world->ball().position() - best_receiving_positions[i]).orientation();
        receiver_positioning_tactics[i]->updateControlParams(
            best_receiving_positions[i], receiver_orientation, 0.0,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);
    }
}

void FreeKickPlayFSM::setReceiverAndDefenderTactics(
    PriorityTacticVector &tactics_to_run, const Update &event, int ideal_num_receivers,
    int num_tactics_already_assigned,
    const std::vector<Point> &existing_receiver_positions,
    const std::optional<Point> &pass_origin_override)
{
    int num_tactics_remaining = std::max(
        0, static_cast<int>(event.common.num_tactics) - num_tactics_already_assigned);
    int num_receivers = std::min(ideal_num_receivers, num_tactics_remaining);
    int num_defenders = std::max(0, num_tactics_remaining - num_receivers);

    if (num_receivers > 0)
    {
        updateReceiverPositioningTactics(event.common.world_ptr, num_receivers,
                                         existing_receiver_positions,
                                         pass_origin_override);
        tactics_to_run[0].insert(tactics_to_run[0].end(),
                                 receiver_positioning_tactics.begin(),
                                 receiver_positioning_tactics.end());
    }

    defense_play->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    if (num_defenders > 0)
    {
        defense_play->updateTactics(PlayUpdate(
            event.common.world_ptr, num_defenders,
            [&tactics_to_run](PriorityTacticVector new_tactics) {
                for (const auto &tactic_vector : new_tactics)
                {
                    tactics_to_run.push_back(tactic_vector);
                }
            },
            event.common.inter_play_communication,
            event.common.set_inter_play_communication_fun));
    }
}

void FreeKickPlayFSM::updateAlignToBallTactic(const WorldPtr &world_ptr)
{
    // Face towards the center of the left segment of the enemy defense area,
    // so we are prepared to take a shot on enemy net, or pass the ball near
    // the enemy defense area.
    Point position_to_face =
        Point(world_ptr->field().enemyDefenseArea().negXNegYCorner().x(), 0.0);
    Point ball_pos           = world_ptr->ball().position();
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
           shot->getOpenAngle() > Angle::fromDegrees(strategy->getAiConfig()
                                                         .attacker_tactic_config()
                                                         .min_open_angle_for_shot_deg());
}

void FreeKickPlayFSM::shootBall(const Update &event)
{
    LOG(INFO) << "Shooting ball.";
    PriorityTacticVector tactics_to_run = {{}};

    Point ball_pos = event.common.world_ptr->ball().position();

    shoot_tactic->updateControlParams(
        {ball_pos, (shot->getPointToShootAt() - ball_pos).orientation(),
         BALL_MAX_SPEED_METERS_PER_SECOND});
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
           strategy->getAiConfig()
               .free_kick_play_config()
               .max_time_commit_to_pass_seconds();
}

void FreeKickPlayFSM::chipBall(const Update &event)
{
    LOG(INFO) << "Time to look for pass expired. Chipping ball.";
    PriorityTacticVector tactics_to_run = {{}};

    Point ball_pos = event.common.world_ptr->ball().position();
    std::optional<Robot> robot_kicking_opt =
        event.common.world_ptr->friendlyTeam().getNearestRobot(ball_pos);
    if (!robot_kicking_opt.has_value())
    {
        LOG(WARNING) << "No robot found to chip the ball during free kick.";
        return;
    }

    // Chip towards the friendly farthest up the enemy half,
    // or the center of the field if no friendly is in the enemy half.
    Point chip_target(0, 0);
    for (const Robot &friendly : event.common.world_ptr->friendlyTeam().getAllRobots())
    {
        // Skip over the robot kicking the ball
        if (friendly.id() == robot_kicking_opt->id())
        {
            continue;
        }

        if (friendly.position().x() > chip_target.x())
        {
            chip_target = friendly.position();
        }
    }

    Point chip_origin  = event.common.world_ptr->ball().position();
    Vector chip_vector = chip_target - chip_origin;

    chip_tactic->updateControlParams(
        {chip_origin, chip_vector.orientation(), chip_vector.length()});
    tactics_to_run[0].emplace_back(chip_tactic);

    event.common.set_tactics(tactics_to_run);
}

void FreeKickPlayFSM::lookForPass(const FreeKickPlayFSM::Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // Keep the kicker aligned to the ball
    updateAlignToBallTactic(event.common.world_ptr);
    tactics_to_run[0].emplace_back(align_to_ball_tactic);

    // Already assigned one tactic to align to ball.
    // Assign up to 2 receivers and the remaining tactics are defenders
    setReceiverAndDefenderTactics(tactics_to_run, event, 2, 1);

    // Find the current best pass
    // Avoid passes to the goalie and the passing robot
    std::vector<RobotId> robots_to_ignore = {};
    auto friendly_goalie_id_opt = event.common.world_ptr->friendlyTeam().getGoalieId();
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
    best_pass_and_score_so_far = strategy->getBestPass(robots_to_ignore);

    event.common.set_tactics(tactics_to_run);
}

bool FreeKickPlayFSM::passFound(const Update &event)
{
    double time_since_pass_optimization_start_seconds =
        (event.common.world_ptr->getMostRecentTimestamp() - pass_optimization_start_time)
            .toSeconds();

    double abs_min_pass_score =
        strategy->getAiConfig().passing_config().abs_min_pass_score();
    double min_perfect_pass_score =
        strategy->getAiConfig().passing_config().min_perfect_pass_score();
    double pass_score_ramp_down_duration =
        strategy->getAiConfig().free_kick_play_config().max_time_commit_to_pass_seconds();

    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    double min_score =
        min_perfect_pass_score - std::min(time_since_pass_optimization_start_seconds /
                                              pass_score_ramp_down_duration,
                                          min_perfect_pass_score - abs_min_pass_score);

    return best_pass_and_score_so_far.rating > min_score;
}

bool FreeKickPlayFSM::shouldAbortPass(const Update &event)
{
    // Check if ball has already been passed
//    if (distance(event.common.world_ptr->ball().position(),
//                 best_pass_and_score_so_far.pass.passerPoint()) >
//        BALL_IN_PLAY_DISTANCE_THRESHOLD_METERS)
//    {
//        return false;
//    }

    // Abort pass if the pass score has dropped significantly
    best_pass_and_score_so_far.rating =
        ratePass(*event.common.world_ptr, best_pass_and_score_so_far.pass,
                 strategy->getAiConfig().passing_config());
    double abs_min_pass_score =
        strategy->getAiConfig().passing_config().abs_min_pass_score();
    return best_pass_and_score_so_far.rating < abs_min_pass_score;
}


void FreeKickPlayFSM::passBall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    Pass pass = best_pass_and_score_so_far.pass;

    passer_tactic->updateControlParams(
        {pass.passerPoint(), pass.passerOrientation(), pass.speed()});
    receiver_tactic->updateControlParams(pass.receiverPoint());
    tactics_to_run[0].emplace_back(passer_tactic);
    tactics_to_run[0].emplace_back(receiver_tactic);

    // Already assigned two tactics to pass and receive.
    // Assign up to 1 other receiver and the remaining tactics are defenders
    setReceiverAndDefenderTactics(tactics_to_run, event, 1, 2, {pass.receiverPoint()},
                                  pass.receiverPoint());

    event.common.set_tactics(tactics_to_run);
}

bool FreeKickPlayFSM::shotDone(const Update &event)
{
    return shoot_tactic->done();
}

bool FreeKickPlayFSM::passDone(const FreeKickPlayFSM::Update &event)
{
    return receiver_tactic->done();
}

bool FreeKickPlayFSM::chipDone(const FreeKickPlayFSM::Update &event)
{
    return chip_tactic->done();
}
