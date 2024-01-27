#include "software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h"

BallPlacementPlayFSM::BallPlacementPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      pivot_kick_tactic(std::make_shared<WallKickoffTactic>(ai_config)),
      place_ball_tactic(std::make_shared<PlaceBallTactic>(ai_config)),
      align_placement_tactic(std::make_shared<PlaceBallMoveTactic>()),
      retreat_tactic(std::make_shared<MoveTactic>()),
      move_tactics(std::vector<std::shared_ptr<PlaceBallMoveTactic>>())
{
}

void BallPlacementPlayFSM::kickOffWall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // setup move tactics for robots away from ball placing robot
    setupMoveTactics(event);
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    // setup wall kickoff tactic for ball placing robot
    Point ball_pos            = event.common.world.ball().position();
    Rectangle field_lines     = event.common.world.field().fieldLines();
    AutoChipOrKick auto_chick = {AutoChipOrKickMode::AUTOKICK,
                                 WALL_KICKOFF_VELOCITY_M_PER_S};

    Angle kick_angle = calculateWallKickoffAngle(ball_pos, field_lines);
    pivot_kick_tactic->updateControlParams(ball_pos, kick_angle, auto_chick);
    tactics_to_run[0].emplace_back(pivot_kick_tactic);

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::alignPlacement(const Update &event)
{
    std::optional<Point> placement_point =
        event.common.world.gameState().getBallPlacementPoint();

    if (placement_point.has_value())
    {
        PriorityTacticVector tactics_to_run = {{}};

        // setup move tactics for robots away from ball placing robot
        setupMoveTactics(event);
        tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                                 move_tactics.end());

        // find position behind the ball where the ball is aligned directly in front
        // placement point from the placing robot's POV
        Vector alignment_vector =
            (placement_point.value() - event.common.world.ball().position()).normalize();
        Angle setup_angle = alignment_vector.orientation();
        setup_point       = event.common.world.ball().position() -
                      2 * alignment_vector * ROBOT_MAX_RADIUS_METERS;

        align_placement_tactic->updateControlParams(
            setup_point, setup_angle, 0.0, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0);

        tactics_to_run[0].emplace_back(align_placement_tactic);

        event.common.set_tactics(tactics_to_run);
    }
}

void BallPlacementPlayFSM::placeBall(const Update &event)
{
    PriorityTacticVector tactics_to_run = {{}};

    // setup move tactics for robots away from ball placing robot
    setupMoveTactics(event);
    tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                             move_tactics.end());

    Angle final_angle = Angle::zero();
    std::optional<Point> placement_point =
        event.common.world.gameState().getBallPlacementPoint();

    Vector placement_dribble_vector;
    if (placement_point.has_value())
    {
        placement_dribble_vector =
            placement_point.value() - event.common.world.ball().position();
        final_angle = placement_dribble_vector.orientation();
    }

    // setup ball placement tactic for ball placing robot
    place_ball_tactic->updateControlParams(
        event.common.world.gameState().getBallPlacementPoint(), final_angle, true);
    tactics_to_run[0].emplace_back(place_ball_tactic);

    event.common.set_tactics(tactics_to_run);
}

void BallPlacementPlayFSM::startWait(const Update &event)
{
    start_time = std::chrono::system_clock::now();
}

void BallPlacementPlayFSM::retreat(const Update &event)
{
    World world = event.common.world;
    std::optional<Robot> nearest_robot =
        world.friendlyTeam().getNearestRobot(world.ball().position());

    if (nearest_robot.has_value())
    {
        PriorityTacticVector tactics_to_run = {{}};

        // setup move tactics for robots away from ball placing robot
        setupMoveTactics(event);
        tactics_to_run[0].insert(tactics_to_run[0].end(), move_tactics.begin(),
                                 move_tactics.end());

        Point ball_pos = world.ball().position();

        // robot will try to retreat backwards from wherever it is currently facing
        Angle final_orientation  = nearest_robot.value().orientation();
        Vector retreat_direction = (nearest_robot->position() - ball_pos).normalize();
        Point retreat_position = ball_pos + retreat_direction * (RETREAT_DISTANCE_METERS +
                                                                 ROBOT_MAX_RADIUS_METERS);

        // if the initial retreat position is out of the field boundary, have it retreat
        // towards the closest goal
        if (!contains(world.field().fieldBoundary(), retreat_position))
        {
            bool in_friendly_half = contains(world.field().friendlyHalf(), ball_pos);
            Point closer_goal     = world.field().friendlyGoalCenter();
            if (!in_friendly_half)
            {
                closer_goal = world.field().enemyGoalCenter();
            }
            retreat_direction = (closer_goal - ball_pos).normalize();
            retreat_position  = ball_pos + retreat_direction * (RETREAT_DISTANCE_METERS +
                                                               ROBOT_MAX_RADIUS_METERS);
        }

        // setup ball placement tactic for ball placing robot
        retreat_tactic->updateControlParams(
            retreat_position, final_orientation, 0.0, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::AVOID, {AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0);
        tactics_to_run[0].emplace_back(retreat_tactic);

        event.common.set_tactics(tactics_to_run);
    }
}

bool BallPlacementPlayFSM::shouldKickOffWall(const Update &event)
{
    // check if ball is too close to border
    Point ball_pos        = event.common.world.ball().position();
    Rectangle field_lines = event.common.world.field().fieldLines();

    return !contains(field_lines, ball_pos);
}

bool BallPlacementPlayFSM::alignDone(const Update &event)
{
    std::optional<Robot> nearest_robot =
        event.common.world.friendlyTeam().getNearestRobot(
            event.common.world.ball().position());
    if (nearest_robot.has_value())
    {
        return comparePoints(nearest_robot.value().position(), setup_point);
    }
    else
    {
        return false;
    }
}

bool BallPlacementPlayFSM::kickDone(const Update &event)
{
    const auto ball_velocity = event.common.world.ball().velocity().length();
    const auto ball_is_kicked_m_per_s_threshold = this->ai_config.ai_parameter_config().ball_is_kicked_m_per_s_threshold();
    return (ball_velocity > ball_is_kicked_m_per_s_threshold) && !shouldKickOffWall(event);
}

bool BallPlacementPlayFSM::ballPlaced(const Update &event)
{
    Point ball_pos = event.common.world.ball().position();
    std::optional<Point> placement_point =
        event.common.world.gameState().getBallPlacementPoint();
    std::vector<Robot> robots = event.common.world.friendlyTeam().getAllRobots();

    // see if the ball is at the placement destination
    if (placement_point.has_value())
    {
        return comparePoints(ball_pos, placement_point.value(),
                             PLACEMENT_DIST_THRESHOLD_METERS) &&
               event.common.world.ball().velocity().length() <
                   this->ai_config.ai_parameter_config()
                       .ball_is_kicked_m_per_s_threshold();
    }
    else
    {
        return true;
    }
}

bool BallPlacementPlayFSM::waitDone(const Update &event)
{
    std::chrono::time_point<std::chrono::system_clock> current_time =
        std::chrono::system_clock::now();
    return static_cast<double>(
               std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time)
                   .count()) > BALL_IS_PLACED_WAIT_S;
}

bool BallPlacementPlayFSM::retreatDone(const Update &event)
{
    Point ball_position = event.common.world.ball().position();
    return distance(ball_position, event.common.world.friendlyTeam()
                                       .getNearestRobot(ball_position)
                                       ->position()) > RETREAT_DISTANCE_METERS;
}

Angle BallPlacementPlayFSM::calculateWallKickoffAngle(const Point &ball_pos,
                                                      const Rectangle &field_lines)
{
    Angle kick_angle;
    if (ball_pos.x() > field_lines.xMax())
    {
        if (ball_pos.y() > 0)
        {
            kick_angle = Angle::fromDegrees(45);
        }
        else
        {
            kick_angle = Angle::fromDegrees(-45);
        }
    }
    else if (ball_pos.x() < field_lines.xMin())
    {
        if (ball_pos.y() > 0)
        {
            kick_angle = Angle::fromDegrees(135);
        }
        else
        {
            kick_angle = Angle::fromDegrees(-135);
        }
    }
    else if (ball_pos.y() > field_lines.yMax())
    {
        if (ball_pos.x() > 0)
        {
            kick_angle = Angle::fromDegrees(135);
        }
        else
        {
            kick_angle = Angle::fromDegrees(45);
        }
    }
    else if (ball_pos.y() < field_lines.yMin())
    {
        if (ball_pos.x() > 0)
        {
            kick_angle = Angle::fromDegrees(-135);
        }
        else
        {
            kick_angle = Angle::fromDegrees(-45);
        }
    }
    return kick_angle;
}

void BallPlacementPlayFSM::setupMoveTactics(const Update &event)
{
    // assign all but one of the robots to line up away from the ball placing robot
    unsigned int num_move_tactics = event.common.num_tactics - 1;

    if (num_move_tactics == 0)
    {
        return;
    }

    move_tactics = std::vector<std::shared_ptr<PlaceBallMoveTactic>>(num_move_tactics);
    std::generate(move_tactics.begin(), move_tactics.end(),
                  [this]() { return std::make_shared<PlaceBallMoveTactic>(); });

    // non goalie and non ball placing robots line up along a line just outside the
    // friendly defense area to wait for ball placement to finish
    Vector waiting_line_vector =
        event.common.world.field().friendlyDefenseArea().posXPosYCorner() -
        event.common.world.field().friendlyDefenseArea().posXNegYCorner();
    Point waiting_line_start_point =
        event.common.world.field().friendlyDefenseArea().posXNegYCorner() +
        Vector(ROBOT_MAX_RADIUS_METERS * 3,
               0);  // Path planner can slow down when pathing through objects - buffer
    // zone of radius x 3 should help

    for (unsigned int i = 0; i < move_tactics.size(); i++)
    {
        Point waiting_destination =
            waiting_line_start_point +
            waiting_line_vector.normalize(waiting_line_vector.length() * i /
                                          static_cast<double>(move_tactics.size()));
        move_tactics.at(i)->updateControlParams(waiting_destination, Angle::zero(), 0.0);
    }
}
