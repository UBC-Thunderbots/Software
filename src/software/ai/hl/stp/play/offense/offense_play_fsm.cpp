#include "software/ai/hl/stp/play/offense/offense_play_fsm.h"

OffensePlayFSM::OffensePlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config_(ai_config),
      attacker_tactic_(std::make_shared<AttackerTactic>(ai_config)),
      receiver_tactic_(
          std::make_shared<ReceiverTactic>(ai_config.receiver_tactic_config())),
      receiver_position_generator_(ReceiverPositionGenerator<EighteenZoneId>(
          std::make_shared<const EighteenZonePitchDivision>(
              Field::createSSLDivisionBField()),
          ai_config.passing_config())),
      pass_generator_(ai_config.passing_config()),
      best_pass_with_rating_(
          PassWithRating{.pass = Pass(Point(), Point(), 0), .rating = 0})
{
}

bool OffensePlayFSM::attackerDone(const Update& event) const
{
    return attacker_tactic_->done();
}

bool OffensePlayFSM::attackerPassing(const Update& event) const
{
    return attacker_tactic_->getCurrentSkill() == AttackerSkill::PASS;
}

bool OffensePlayFSM::shouldAbortPass(const Update& event)
{
    const Point ball_position  = event.common.world_ptr->ball().position();
    const Point passer_point   = best_pass_with_rating_.pass.passerPoint();
    const Point receiver_point = best_pass_with_rating_.pass.receiverPoint();
    const double short_pass_threshold =
        ai_config_.shoot_or_pass_play_config().short_pass_threshold();

    if ((receiver_point - passer_point).length() >= short_pass_threshold)
    {
        const Polygon pass_area_polygon =
            Polygon::fromSegment(Segment(passer_point, receiver_point), 0.5);

        // If the ball is outside of the "passing area", we should abort since the ball
        // has likely strayed from the intended pass trajectory
        if (!contains(pass_area_polygon, ball_position))
        {
            return true;
        }
    }

    const double ball_velocity = event.common.world_ptr->ball().velocity().length();
    const double ball_shot_threshold =
        ai_config_.shoot_or_pass_play_config().ball_shot_threshold();
    const double min_distance_to_pass =
        ai_config_.shoot_or_pass_play_config().min_distance_to_pass();

    // If ball is not moving and is too far away from pass origin point, abort the pass
    if ((ball_velocity < ball_shot_threshold) &&
        ((ball_position - passer_point).length() > min_distance_to_pass))
    {
        return true;
    }

    // If we haven't taken the pass yet (i.e. attacker is not done), abort the pass
    // if its rating gets too low
    if (!attacker_tactic_->done())
    {
        const double abs_min_pass_score =
            ai_config_.shoot_or_pass_play_config().abs_min_pass_score();

        best_pass_with_rating_.rating =
            ratePass(*event.common.world_ptr, best_pass_with_rating_.pass,
                     ai_config_.passing_config());

        return best_pass_with_rating_.rating < abs_min_pass_score;
    }

    return false;
}

bool OffensePlayFSM::passReceived(const Update& event) const
{
    return receiver_tactic_->done();
}

void OffensePlayFSM::attack(const Update& event)
{
    PriorityTacticVector tactics = {{attacker_tactic_, receiver_tactic_}, {}};

    if (event.common.num_tactics > 2)
    {
        std::vector<Point> existing_receiver_positions = {
            best_pass_with_rating_.pass.receiverPoint()};

        updateOffensivePositioningTactics(event.common.world_ptr,
                                          event.common.num_tactics - 2,
                                          existing_receiver_positions);

        tactics[1].insert(tactics[1].end(), offensive_positioning_tactics_.begin(),
                          offensive_positioning_tactics_.end());
    }

    event.common.set_tactics(tactics);
}

void OffensePlayFSM::receive(const Update& event)
{
    PriorityTacticVector tactics = {{receiver_tactic_}, {}};

    std::vector<Point> existing_receiver_positions = {
        best_pass_with_rating_.pass.receiverPoint()};

    updateOffensivePositioningTactics(
        event.common.world_ptr, event.common.num_tactics - 1, existing_receiver_positions,
        best_pass_with_rating_.pass.receiverPoint());

    tactics[1].insert(tactics[1].end(), offensive_positioning_tactics_.begin(),
                      offensive_positioning_tactics_.end());

    event.common.set_tactics(tactics);
}

void OffensePlayFSM::resetTactics(const Update& event)
{
    // Avoid passes to the goalie and the passing robot
    std::vector<RobotId> robots_to_ignore = {};

    auto friendly_goalie_id = event.common.world_ptr->friendlyTeam().getGoalieId();
    if (friendly_goalie_id.has_value())
    {
        robots_to_ignore.push_back(friendly_goalie_id.value());
    }

    auto robot_with_ball = event.common.world_ptr->friendlyTeam().getNearestRobot(
        event.common.world_ptr->ball().position());
    if (robot_with_ball.has_value())
    {
        robots_to_ignore.push_back(robot_with_ball.value().id());
    }

    best_pass_with_rating_ =
        pass_generator_.getBestPass(*event.common.world_ptr, robots_to_ignore);

    attacker_tactic_->updateControlParams(best_pass_with_rating_.pass);
    attacker_tactic_->selectSkill(event.common.world_ptr);

    receiver_tactic_ =
        std::make_shared<ReceiverTactic>(ai_config_.receiver_tactic_config());
    receiver_tactic_->updateControlParams(best_pass_with_rating_.pass);
}

void OffensePlayFSM::terminate(const Terminate& event)
{
    attacker_tactic_->terminate(event.world_ptr);
}

void OffensePlayFSM::updateOffensivePositioningTactics(
    const WorldPtr world, unsigned int num_tactics,
    const std::vector<Point>& existing_receiver_positions,
    const std::optional<Point>& pass_origin_override)
{
    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    if (num_tactics != offensive_positioning_tactics_.size())
    {
        offensive_positioning_tactics_ =
            std::vector<std::shared_ptr<MoveTactic>>(num_tactics);
        std::generate(offensive_positioning_tactics_.begin(),
                      offensive_positioning_tactics_.end(),
                      []() { return std::make_shared<MoveTactic>(); });
    }

    std::vector<Point> best_receiving_positions =
        receiver_position_generator_.getBestReceivingPositions(
            *world, num_tactics, existing_receiver_positions, pass_origin_override);

    // Note that getBestReceivingPositions may return fewer positions than requested
    // if there are not enough robots, so we will need to check the size of the vector.
    for (unsigned int i = 0;
         i < offensive_positioning_tactics_.size() && i < best_receiving_positions.size();
         i++)
    {
        Angle receiver_orientation =
            (world->ball().position() - best_receiving_positions[i]).orientation();
        offensive_positioning_tactics_[i]->updateControlParams(
            best_receiving_positions[i], receiver_orientation,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);
    }
}
