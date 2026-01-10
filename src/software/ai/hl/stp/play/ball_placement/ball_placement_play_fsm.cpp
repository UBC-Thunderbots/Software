#include "software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h"

BallPlacementPlayFSM::BallPlacementPlayFSM(
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayFSM<BallPlacementPlayFSM>(ai_config_ptr),
      align_wall_tactic(std::make_shared<BallPlacementMoveTactic>(ai_config_ptr)),
      pickoff_wall_tactic(std::make_shared<BallPlacementDribbleTactic>(ai_config_ptr)),
      place_ball_tactic(std::make_shared<BallPlacementDribbleTactic>(ai_config_ptr)),
      align_placement_tactic(std::make_shared<BallPlacementMoveTactic>(ai_config_ptr)),
      retreat_tactic(std::make_shared<MoveTactic>(ai_config_ptr)),
      wait_tactic(std::make_shared<MoveTactic>(ai_config_ptr)),
      move_tactics(std::vector<std::shared_ptr<BallPlacementMoveTactic>>())
{
}

void BallPlacementPlayFSM::alignWall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{align_wall_tactic}};

    setPickOffDest(event);

    align_wall_tactic->updateControlParams(
        pickoff_point, pickoff_final_orientation, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);

    setupMoveTactics(event, event.common.num_tactics -
                                static_cast<unsigned int>(tactics_to_run[0].size()));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::setPickOffDest(const Update &event)
{
    Point ball_pos           = event.common.world_ptr->ball().position();
    Rectangle field_boundary = event.common.world_ptr->field().fieldBoundary();

    auto [orientation, dest]  = calculateWallPickOffDest(ball_pos, field_boundary);
    pickoff_final_orientation = orientation;
    pickoff_destination       = dest;
}

void BallPlacementPlayFSM::pickOffWall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{pickoff_wall_tactic}};

    pickoff_wall_tactic->updateControlParams(
        pickoff_destination, pickoff_final_orientation, true,
        TbotsProto::MaxAllowedSpeedMode::BALL_PLACEMENT_WALL_DRIBBLE,
        TbotsProto::MaxAllowedSpeedMode::BALL_PLACEMENT_WALL_DRIBBLE);

    setupMoveTactics(event, event.common.num_tactics -
                                static_cast<unsigned int>(tactics_to_run[0].size()));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::alignPlacement(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();

    if (placement_point.has_value())
    {
        // find position behind the ball where the ball is aligned directly in front
        // placement point from the placing robot's POV
        Vector alignment_vector =
            (placement_point.value() - event.common.world_ptr->ball().position())
                .normalize(ALIGNMENT_VECTOR_LENGTH_M);
        Angle setup_angle = alignment_vector.orientation();
        setup_point       = event.common.world_ptr->ball().position() - alignment_vector;

        align_placement_tactic->updateControlParams(
            setup_point, setup_angle, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::DRIBBLE,
            TbotsProto::ObstacleAvoidanceMode::SAFE);

        tactics_to_run[0].push_back(align_placement_tactic);
    }

    setupMoveTactics(event, event.common.num_tactics -
                                static_cast<unsigned int>(tactics_to_run[0].size()));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::placeBall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();

    if (placement_point.has_value())
    {
        Point ball_pos = event.common.world_ptr->ball().position();
        std::optional<Robot> robot_placing_ball =
            event.common.world_ptr->friendlyTeam().getNearestRobot(ball_pos);

        if (robot_placing_ball.has_value())
        {
            Vector placement_vector =
                placement_point.value() - event.common.world_ptr->ball().position();

            Angle final_angle;
            if (placement_vector.length() > APPROACHING_PLACEMENT_DIST_THRESHOLD_M)
            {
                // We are still far away from the placement point, so just point
                // towards it
                final_angle = placement_vector.orientation();
            }
            else
            {
                // We are near the placement point; keep the same orientation
                // so we don't switch direction (in case the ball rolls slightly
                // past the placement point)
                final_angle = robot_placing_ball->orientation();
            }

            place_ball_tactic->updateControlParams(
                event.common.world_ptr->gameState().getBallPlacementPoint(), final_angle,
                true, TbotsProto::MaxAllowedSpeedMode::DRIBBLE);

            tactics_to_run[0].push_back(place_ball_tactic);
        }
    }

    setupMoveTactics(event, event.common.num_tactics -
                                static_cast<unsigned int>(tactics_to_run[0].size()));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::startWait(const Update &event)
{
    start_time = event.common.world_ptr->getMostRecentTimestamp();
}

void BallPlacementPlayFSM::releaseBall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    WorldPtr world_ptr = event.common.world_ptr;
    std::optional<Robot> nearest_robot =
        world_ptr->friendlyTeam().getNearestRobot(world_ptr->ball().position());

    if (nearest_robot.has_value())
    {
        wait_tactic->updateControlParams(
            nearest_robot->position(), nearest_robot->orientation(),
            TbotsProto::DribblerMode::RELEASE_BALL_SLOW,
            TbotsProto::BallCollisionType::ALLOW, {AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::BALL_PLACEMENT_RETREAT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);

        tactics_to_run[0].push_back(wait_tactic);
    }

    setupMoveTactics(event, event.common.num_tactics -
                                static_cast<unsigned int>(tactics_to_run[0].size()));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::retreat(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    WorldPtr world_ptr = event.common.world_ptr;
    std::optional<Robot> nearest_robot =
        world_ptr->friendlyTeam().getNearestRobot(world_ptr->ball().position());

    if (nearest_robot.has_value())
    {
        Point ball_pos = world_ptr->ball().position();

        // Robot will try to retreat backwards from wherever it is currently facing
        Angle final_orientation = nearest_robot->orientation();
        Vector retreat_dir      = nearest_robot->position() - ball_pos;
        Point retreat_pos       = ball_pos + retreat_dir.normalize(RETREAT_DISTANCE_M);

        // If the initial retreat position is out of the field boundary, have it retreat
        // towards the closest goal
        if (!contains(world_ptr->field().fieldBoundary(), retreat_pos))
        {
            Point closer_goal;
            if (contains(world_ptr->field().friendlyHalf(), ball_pos))
            {
                closer_goal = world_ptr->field().friendlyGoalCenter();
            }
            else
            {
                closer_goal = world_ptr->field().enemyGoalCenter();
            }

            retreat_dir = closer_goal - ball_pos;
            retreat_pos = ball_pos + retreat_dir.normalize(RETREAT_DISTANCE_M);
        }

        retreat_tactic->updateControlParams(
            retreat_pos, final_orientation, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::BALL_PLACEMENT_RETREAT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);

        tactics_to_run[0].push_back(retreat_tactic);
    }

    setupMoveTactics(event, event.common.num_tactics -
                                static_cast<unsigned int>(tactics_to_run[0].size()));
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    event.common.set_tactics(tactics_to_run);
}

bool BallPlacementPlayFSM::shouldPickOffWall(const Update &event)
{
    const Point ball_pos        = event.common.world_ptr->ball().position();
    const Rectangle field_lines = event.common.world_ptr->field().fieldBoundary();
    return std::abs(signedDistance(ball_pos, field_lines)) <=
           MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;
}

bool BallPlacementPlayFSM::alignDone(const Update &event)
{
    std::optional<Robot> nearest_robot =
        event.common.world_ptr->friendlyTeam().getNearestRobot(
            event.common.world_ptr->ball().position());

    if (nearest_robot.has_value())
    {
        return comparePoints(nearest_robot->position(), setup_point);
    }

    return false;
}

bool BallPlacementPlayFSM::wallAlignDone(const Update &event)
{
    std::optional<Robot> nearest_robot =
        event.common.world_ptr->friendlyTeam().getNearestRobot(
            event.common.world_ptr->ball().position());

    if (nearest_robot.has_value())
    {
        return comparePoints(nearest_robot->position(), pickoff_point);
    }

    return false;
}

bool BallPlacementPlayFSM::wallPickOffDone(const Update &event)
{
    return pickoff_wall_tactic->done();
}

bool BallPlacementPlayFSM::ballPlaced(const Update &event)
{
    const Point ball_pos    = event.common.world_ptr->ball().position();
    const double ball_speed = event.common.world_ptr->ball().velocity().length();

    std::optional<Point> placement_point =
        event.common.world_ptr->gameState().getBallPlacementPoint();
    std::vector<Robot> robots = event.common.world_ptr->friendlyTeam().getAllRobots();

    if (placement_point.has_value())
    {
        const bool is_ball_at_placement_point =
            comparePoints(ball_pos, placement_point.value(), PLACEMENT_DIST_THRESHOLD_M);

        const bool is_ball_stationary =
            ball_speed <
            ai_config_ptr->ai_parameter_config().ball_is_kicked_m_per_s_threshold();

        return is_ball_at_placement_point && is_ball_stationary;
    }

    return true;
}

bool BallPlacementPlayFSM::waitDone(const Update &event)
{
    const Timestamp current_time = event.common.world_ptr->getMostRecentTimestamp();
    return (current_time - start_time) > Duration::fromSeconds(BALL_IS_PLACED_WAIT_S);
}

bool BallPlacementPlayFSM::retreatDone(const Update &event)
{
    return retreat_tactic->done();
}

std::pair<Angle, Point> BallPlacementPlayFSM::calculateWallPickOffDest(
    const Point &ball_pos, const Rectangle &field_boundary)
{
    Angle facing_angle;
    Point backoff_point;

    const bool near_positive_y_boundary =
        (field_boundary.yMax() - ball_pos.y()) < MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;
    const bool near_negative_y_boundary =
        (ball_pos.y() - field_boundary.yMin()) < MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;
    const bool near_positive_x_boundary =
        (field_boundary.xMax() - ball_pos.x()) < MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;
    const bool near_negative_x_boundary =
        (ball_pos.x() - field_boundary.xMin()) < MIN_DISTANCE_FROM_WALL_FOR_ALIGN_M;

    if (near_positive_y_boundary && near_positive_x_boundary)
    {
        facing_angle = Angle::fromDegrees(45);
        backoff_point =
            field_boundary.posXPosYCorner() -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_CORNER_M);
    }
    else if (near_positive_y_boundary && near_negative_x_boundary)
    {
        facing_angle = Angle::fromDegrees(135);
        backoff_point =
            field_boundary.negXPosYCorner() -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_CORNER_M);
    }
    else if (near_negative_y_boundary && near_positive_x_boundary)
    {
        facing_angle = Angle::fromDegrees(-45);
        backoff_point =
            field_boundary.posXNegYCorner() -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_CORNER_M);
    }
    else if (near_negative_y_boundary && near_negative_x_boundary)
    {
        facing_angle = Angle::fromDegrees(-135);
        backoff_point =
            field_boundary.negXNegYCorner() -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_CORNER_M);
    }
    else if (near_positive_y_boundary)
    {
        facing_angle = Angle::fromDegrees(90);
        backoff_point =
            Point(ball_pos.x(), field_boundary.yMax()) -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_WALL_M);
    }
    else if (near_positive_x_boundary)
    {
        facing_angle = Angle::fromDegrees(0);
        backoff_point =
            Point(field_boundary.xMax(), ball_pos.y()) -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_WALL_M);
    }
    else if (near_negative_y_boundary)
    {
        facing_angle = Angle::fromDegrees(-90);
        backoff_point =
            Point(ball_pos.x(), field_boundary.yMin()) -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_WALL_M);
    }
    else if (near_negative_x_boundary)
    {
        facing_angle = Angle::fromDegrees(180);
        backoff_point =
            Point(field_boundary.xMin(), ball_pos.y()) -
            Vector::createFromAngle(facing_angle).normalize(BACK_AWAY_FROM_WALL_M);
    }

    return {facing_angle, backoff_point};
}

void BallPlacementPlayFSM::setupMoveTactics(const Update &event, unsigned int num_tactics)
{
    if (move_tactics.size() != num_tactics)
    {
        move_tactics = std::vector<std::shared_ptr<BallPlacementMoveTactic>>(num_tactics);
        std::generate(
            move_tactics.begin(), move_tactics.end(),
            [&] { return std::make_shared<BallPlacementMoveTactic>(ai_config_ptr); });
    }

    Vector waiting_line_vector =
        event.common.world_ptr->field().friendlyDefenseArea().posXPosYCorner() -
        event.common.world_ptr->field().friendlyDefenseArea().posXNegYCorner();

    Point waiting_line_start_point =
        event.common.world_ptr->field().friendlyDefenseArea().posXNegYCorner() +
        Vector(WAITING_LINE_OFFSET_M, 0);

    for (unsigned int i = 0; i < move_tactics.size(); i++)
    {
        Point waiting_destination =
            waiting_line_start_point +
            waiting_line_vector.normalize(waiting_line_vector.length() * i /
                                          static_cast<double>(move_tactics.size()));

        move_tactics.at(i)->updateControlParams(
            waiting_destination, Angle::zero(),
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::SAFE);
    }
}
