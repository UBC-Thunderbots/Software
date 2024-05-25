#include "software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play_fsm.h"

EnemyBallPlacementPlayFSM::EnemyBallPlacementPlayFSM(TbotsProto::AiConfig ai_config)
    : ai_config(ai_config),
      crease_defender_tactics({std::make_shared<CreaseDefenderTactic>(
                                   ai_config.robot_navigation_obstacle_config()),
                               std::make_shared<CreaseDefenderTactic>(
                                   ai_config.robot_navigation_obstacle_config())}),
      avoid_interference_tactics({
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
          std::make_shared<AvoidInterferenceTactic>(),
      }),
      move_tactics({
          std::make_shared<MoveTactic>(),
          std::make_shared<MoveTactic>(),
          std::make_shared<MoveTactic>(),
      }),
      goalie_tactic(std::make_shared<GoalieTactic>(ai_config)),
      distance_to_keep_meters(
          ai_config.enemy_ball_placement_play_config().distance_to_keep_meters()),
      nearly_placed_threshold_meters(0.5)
{
}

bool EnemyBallPlacementPlayFSM::hasPlacementPoint(const Update& event)
{
    return event.common.world_ptr->gameState().getBallPlacementPoint().has_value();
}

void EnemyBallPlacementPlayFSM::setPlacementPoint(const Update& event)
{
    placement_point = event.common.world_ptr->gameState().getBallPlacementPoint().value();
}

bool EnemyBallPlacementPlayFSM::isNearlyPlaced(const Update& event)
{
    return (event.common.world_ptr->ball().position() - placement_point).length() <
           nearly_placed_threshold_meters;
}

void EnemyBallPlacementPlayFSM::avoid(const Update& event)
{
    WorldPtr world_ptr                  = event.common.world_ptr;
    PriorityTacticVector tactics_to_run = {{}};
    Point ball_pos                      = world_ptr->ball().position();

    Stadium stadium = Stadium(ball_pos, placement_point, distance_to_keep_meters);
    Vector placement_point_to_ball = ball_pos - placement_point;

    // Check if robots are inside the ball placement stadium
    int idx = 0;
    for (auto const& robot : world_ptr->friendlyTeam().getAllRobots())
    {
        if (contains(stadium, robot.position()))
        {
            // The "backbone" of the stadium connects the ball position and the placement
            // point. A robot located in the stadium will move perpendicular to the
            // backbone to be outside the stadium. There are two possible destination
            // points, one on each side of the stadium

            // This unit vector is perpendicular to the backbone, and determines the
            // direction of robot motion
            Vector lateral_positioning_unit_vector =
                placement_point_to_ball.perpendicular().normalize();
            // This vector represents the robot's longitudinal position along the
            // backbone, relative to the placement point
            Vector longitudinal_positioning_vector =
                (robot.position() - placement_point).project(placement_point_to_ball);
            Point p1 = placement_point + longitudinal_positioning_vector +
                       lateral_positioning_unit_vector * distance_to_keep_meters;
            Point p2 = placement_point + longitudinal_positioning_vector -
                       lateral_positioning_unit_vector * distance_to_keep_meters;
            Point destination;
            Rectangle fieldLines = world_ptr->field().fieldLines();
            // If either destination point is outside the field, then pick the other
            // point. It is impossible for both points to be outside the field
            if (!contains(fieldLines, p2))
            {
                destination = p1;
            }
            else if (!contains(fieldLines, p1))
            {
                destination = p2;
            }
            // If both points are in the field, then pick the one closer to the robot
            else if (distance(p1, robot.position()) < distance(p2, robot.position()))
            {
                destination = p1;
            }
            else
            {
                destination = p2;
            }
            // Move to destination point while aligning to the ball
            avoid_interference_tactics[idx]->updateControlParams(
                destination, (ball_pos - robot.position()).orientation(), 0);
        }
        else
        {
            // Stay in place while aligning to the ball
            avoid_interference_tactics[idx]->updateControlParams(
                robot.position(), (ball_pos - robot.position()).orientation(), 0);
        }
        tactics_to_run[0].emplace_back(avoid_interference_tactics[idx]);
        idx++;
    }

    // Set tactics to run
    event.common.set_tactics(tactics_to_run);
}

void EnemyBallPlacementPlayFSM::enterDefensiveFormation(const Update& event)
{
    WorldPtr world_ptr                  = event.common.world_ptr;
    PriorityTacticVector tactics_to_run = {{}};
    Point ball_pos                      = world_ptr->ball().position();

    // Create crease defenders
    crease_defender_tactics[0]->updateControlParams(
        world_ptr->ball().position(), TbotsProto::CreaseDefenderAlignment::LEFT);
    crease_defender_tactics[1]->updateControlParams(
        world_ptr->ball().position(), TbotsProto::CreaseDefenderAlignment::RIGHT);
    tactics_to_run[0].emplace_back(crease_defender_tactics[0]);
    tactics_to_run[0].emplace_back(crease_defender_tactics[1]);

    // Create goalie
    tactics_to_run[0].emplace_back(goalie_tactic);

    // Create move tactics
    Vector positioning_vector = world_ptr->field().friendlyGoalCenter() - ball_pos;
    positioning_vector        = positioning_vector.normalize() * distance_to_keep_meters;

    Vector left_vector  = positioning_vector.rotate(Angle::fromDegrees(-30));
    Vector right_vector = positioning_vector.rotate(Angle::fromDegrees(30));

    Point center = world_ptr->ball().position() + positioning_vector;
    Point left   = world_ptr->ball().position() + left_vector;
    Point right  = world_ptr->ball().position() + right_vector;

    move_tactics[0]->updateControlParams(
        center, positioning_vector.orientation() + Angle::half(), 0);
    move_tactics[1]->updateControlParams(left, left_vector.orientation() + Angle::half(),
                                         0);
    move_tactics[2]->updateControlParams(right,
                                         right_vector.orientation() + Angle::half(), 0);
    tactics_to_run[0].emplace_back(move_tactics[0]);
    tactics_to_run[0].emplace_back(move_tactics[1]);
    tactics_to_run[0].emplace_back(move_tactics[2]);

    // Set tactics to run
    event.common.set_tactics(tactics_to_run);
}
